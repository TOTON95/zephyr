/*
 * Copyright (c) 2026 3DR, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * ST M95P family SPI Page EEPROM driver (DS13999).
 * Covers M95P08 (1 MB), M95P16 (2 MB), M95P32 (4 MB).
 *
 * All variants share the same command set and geometry ratios:
 *   page 512 B · sector 4 KB · block 64 KB
 * Total capacity and JEDEC ID come from DTS (size, jedec-id properties).
 *
 * Flash API:
 *   write_block_size = 1  (NVS-compatible)
 *   erase granularity = 4 KB sector
 *   erase_value = 0xFF
 *   no_explicit_erase = 1  (internal read-modify-write; pre-erase not required)
 */

#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(flash_m95pxx, CONFIG_FLASH_LOG_LEVEL);

#define DT_DRV_COMPAT st_m95pxx

/* Opcodes — identical across the M95P family (DS13999 Rev 2, Section 6) */
#define M95P_OP_READ  0x03 /* Read Data */
#define M95P_OP_WRITE 0x02 /* Page Write (auto erase+program, within one page) */
#define M95P_OP_WREN  0x06 /* Write Enable */
#define M95P_OP_RDSR  0x05 /* Read Status Register */
#define M95P_OP_RDID  0x9F /* Read JEDEC Identification */
#define M95P_OP_PE    0xDB /* Page Erase  (512 B) */
#define M95P_OP_SE    0x20 /* Sector Erase (4 KB) */
#define M95P_OP_BE    0xD8 /* Block Erase  (64 KB) */
#define M95P_OP_CE    0xC7 /* Chip Erase */
#define M95P_OP_RDP   0xAB /* Release from Deep Power-Down */
/* TODO (CONFIG_PM_DEVICE): 0xB9 = Deep Power-Down entry */

/* Status Register */
#define M95P_SR_WIP BIT(0) /* Write In Progress */

/* Fixed geometry — same for all M95P variants */
#define M95P_PAGE_SIZE   512U
#define M95P_SECTOR_SIZE (4U * 1024U)
#define M95P_BLOCK_SIZE  (64U * 1024U)

/*
 * Timeouts: DS13999 Rev 2 Table 26 typ values × 4 safety margin.
 */
#define M95P_WRITE_TIMEOUT_MS 10  /* page write  typ 2 ms   */
#define M95P_PAGE_ERASE_MS    10  /* page erase  typ 1.1 ms */
#define M95P_ERASE_SECTOR_MS  20  /* sector erase typ 1.3 ms */
#define M95P_ERASE_BLOCK_MS   50  /* block erase  typ 4 ms  */
#define M95P_ERASE_CHIP_MS    100 /* chip erase   typ 4 ms  */

#define INST_HAS_WP_OR(n)     DT_INST_NODE_HAS_PROP(n, wp_gpios) ||
#define ANY_INST_HAS_WP_GPIOS DT_INST_FOREACH_STATUS_OKAY(INST_HAS_WP_OR) 0

#define BUF_SET(_name, _arr)                                                                       \
	const struct spi_buf_set _name = {.buffers = (_arr), .count = ARRAY_SIZE(_arr)}

struct m95p_config {
	struct spi_dt_spec spi;
	uint32_t chip_size;  /* from DTS: size property (bytes) */
	uint8_t jedec_id[3]; /* from DTS: jedec-id property     */
#if ANY_INST_HAS_WP_GPIOS
	const struct gpio_dt_spec *wp;
#endif
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	struct flash_pages_layout layout;
#endif
};

struct m95p_data {
	struct k_sem lock;
};

static const struct flash_parameters m95p_params = {
	.write_block_size = 1,
	.erase_value = 0xFF,
	.caps = {.no_explicit_erase = 1},
};

static void m95p_acquire(const struct device *dev)
{
	k_sem_take(&((struct m95p_data *)dev->data)->lock, K_FOREVER);
}

static void m95p_release_sem(const struct device *dev)
{
	k_sem_give(&((struct m95p_data *)dev->data)->lock);
}

static int m95p_cmd1(const struct device *dev, uint8_t op)
{
	const struct m95p_config *cfg = dev->config;
	const struct spi_buf tx[] = {{.buf = &op, .len = 1}};

	BUF_SET(tx_s, tx);

	return spi_write_dt(&cfg->spi, &tx_s) != 0 ? -EIO : 0;
}

static int m95p_read_sr(const struct device *dev, uint8_t *sr)
{
	const struct m95p_config *cfg = dev->config;
	const uint8_t op = M95P_OP_RDSR;
	const struct spi_buf tx[] = {{.buf = (void *)&op, .len = 1}};
	const struct spi_buf rx[] = {{.len = 1}, {.buf = sr, .len = 1}};

	BUF_SET(tx_s, tx);
	BUF_SET(rx_s, rx);

	return spi_transceive_dt(&cfg->spi, &tx_s, &rx_s) != 0 ? -EIO : 0;
}

static int m95p_wait_ready(const struct device *dev, int timeout_ms)
{
	int64_t deadline = k_uptime_get() + timeout_ms;
	uint8_t sr;
	int err;

	do {
		err = m95p_read_sr(dev, &sr);
		if (err != 0) {
			return -EIO;
		}
		if (!(sr & M95P_SR_WIP)) {
			return 0;
		}
		k_msleep(1);
	} while (k_uptime_get() < deadline);

	return -ETIMEDOUT;
}

static int m95p_wren(const struct device *dev)
{
	return m95p_cmd1(dev, M95P_OP_WREN);
}

static int m95p_read(const struct device *dev, off_t offset, void *data, size_t len)
{
	const struct m95p_config *cfg = dev->config;
	int err;

	if (offset < 0 || (uint32_t)offset + len > cfg->chip_size) {
		return -EINVAL;
	}
	if (len == 0) {
		return 0;
	}

	uint8_t hdr[] = {
		M95P_OP_READ,
		(offset >> 16) & 0xFF,
		(offset >> 8) & 0xFF,
		(offset >> 0) & 0xFF,
	};
	const struct spi_buf tx[] = {{.buf = hdr, .len = sizeof(hdr)}};
	const struct spi_buf rx[] = {
		{.len = sizeof(hdr)},
		{.buf = data, .len = len},
	};

	BUF_SET(tx_s, tx);
	BUF_SET(rx_s, rx);

	m95p_acquire(dev);
	err = spi_transceive_dt(&cfg->spi, &tx_s, &rx_s);
	m95p_release_sem(dev);

	return err != 0 ? -EIO : 0;
}

#if ANY_INST_HAS_WP_GPIOS
static void m95p_wp_set(const struct device *dev, int val)
{
	const struct m95p_config *cfg = dev->config;

	if (cfg->wp) {
		gpio_pin_set_dt(cfg->wp, val);
	}
}
#endif

static int m95p_write_page(const struct device *dev, off_t offset, const void *data, size_t len)
{
	const struct m95p_config *cfg = dev->config;
	int err;
	uint8_t hdr[] = {
		M95P_OP_WRITE,
		(offset >> 16) & 0xFF,
		(offset >> 8) & 0xFF,
		(offset >> 0) & 0xFF,
	};
	const struct spi_buf tx[] = {
		{.buf = hdr, .len = sizeof(hdr)},
		{.buf = (void *)data, .len = len},
	};

	BUF_SET(tx_s, tx);

	err = m95p_wren(dev);
	if (err != 0) {
		return err;
	}
	err = spi_write_dt(&cfg->spi, &tx_s);
	if (err != 0) {
		return -EIO;
	}
	return m95p_wait_ready(dev, M95P_WRITE_TIMEOUT_MS);
}

static int m95p_write(const struct device *dev, off_t offset, const void *data, size_t len)
{
	const struct m95p_config *cfg = dev->config;
	int err = 0;

	if (offset < 0 || (uint32_t)offset + len > cfg->chip_size) {
		return -EINVAL;
	}
	if (len == 0) {
		return 0;
	}

	m95p_acquire(dev);

#if ANY_INST_HAS_WP_GPIOS
	m95p_wp_set(dev, 0); /* deassert WP (active-low) → writes allowed */
#endif

	while (len > 0 && err == 0) {
		size_t page_rem = M95P_PAGE_SIZE - ((size_t)offset % M95P_PAGE_SIZE);
		size_t chunk = MIN(len, page_rem);

		err = m95p_write_page(dev, offset, data, chunk);
		offset += chunk;
		data = (const uint8_t *)data + chunk;
		len -= chunk;
	}

#if ANY_INST_HAS_WP_GPIOS
	m95p_wp_set(dev, 1); /* re-assert WP */
#endif

	m95p_release_sem(dev);
	return err;
}

static int m95p_erase_op(const struct device *dev, uint8_t opcode, off_t offset, int timeout_ms)
{
	const struct m95p_config *cfg = dev->config;
	int err;
	uint8_t hdr[] = {
		opcode,
		(offset >> 16) & 0xFF,
		(offset >> 8) & 0xFF,
		(offset >> 0) & 0xFF,
	};
	const struct spi_buf tx[] = {{.buf = hdr, .len = sizeof(hdr)}};

	BUF_SET(tx_s, tx);

	err = m95p_wren(dev);
	if (err != 0) {
		return err;
	}
	err = spi_write_dt(&cfg->spi, &tx_s);
	if (err != 0) {
		return -EIO;
	}
	return m95p_wait_ready(dev, timeout_ms);
}

static int m95p_erase(const struct device *dev, off_t offset, size_t size)
{
	const struct m95p_config *cfg = dev->config;
	int err = 0;

	if (offset < 0 || (uint32_t)offset + size > cfg->chip_size) {
		return -EINVAL;
	}
	if (((size_t)offset % M95P_SECTOR_SIZE) != 0 || (size % M95P_SECTOR_SIZE) != 0) {
		return -EINVAL;
	}

	m95p_acquire(dev);

#if ANY_INST_HAS_WP_GPIOS
	m95p_wp_set(dev, 0);
#endif

	if (size == cfg->chip_size) {
		err = m95p_wren(dev);
		if (err == 0) {
			err = m95p_cmd1(dev, M95P_OP_CE);
			if (err == 0) {
				err = m95p_wait_ready(dev, M95P_ERASE_CHIP_MS);
			}
		}
		goto done;
	}

	while (size > 0 && err == 0) {
		if (size >= M95P_BLOCK_SIZE && ((size_t)offset % M95P_BLOCK_SIZE) == 0) {
			err = m95p_erase_op(dev, M95P_OP_BE, offset, M95P_ERASE_BLOCK_MS);
			offset += M95P_BLOCK_SIZE;
			size -= M95P_BLOCK_SIZE;
		} else {
			err = m95p_erase_op(dev, M95P_OP_SE, offset, M95P_ERASE_SECTOR_MS);
			offset += M95P_SECTOR_SIZE;
			size -= M95P_SECTOR_SIZE;
		}
	}

done:
#if ANY_INST_HAS_WP_GPIOS
	m95p_wp_set(dev, 1);
#endif
	m95p_release_sem(dev);
	return err;
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
static void m95p_pages_layout(const struct device *dev, const struct flash_pages_layout **layout,
			      size_t *layout_size)
{
	const struct m95p_config *cfg = dev->config;

	*layout = &cfg->layout;
	*layout_size = 1;
}
#endif

static const struct flash_parameters *m95p_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);
	return &m95p_params;
}

static int m95p_get_size(const struct device *dev, uint64_t *size)
{
	const struct m95p_config *cfg = dev->config;

	*size = cfg->chip_size;
	return 0;
}

#if !defined(CONFIG_FLASH_M95PXX_SKIP_JEDEC_CHECK)
static int m95p_check_jedec(const struct device *dev)
{
	const struct m95p_config *cfg = dev->config;
	const uint8_t op = M95P_OP_RDID;
	uint8_t id[3] = {0};
	const struct spi_buf tx[] = {{.buf = (void *)&op, .len = 1}};
	const struct spi_buf rx[] = {
		{.len = 1},
		{.buf = id, .len = sizeof(id)},
	};

	BUF_SET(tx_s, tx);
	BUF_SET(rx_s, rx);

	if (spi_transceive_dt(&cfg->spi, &tx_s, &rx_s) != 0) {
		return -EIO;
	}
	if (memcmp(id, cfg->jedec_id, sizeof(id)) != 0) {
		LOG_ERR("Wrong JEDEC ID: %02X %02X %02X, expected %02X %02X %02X", id[0], id[1],
			id[2], cfg->jedec_id[0], cfg->jedec_id[1], cfg->jedec_id[2]);
		return -ENODEV;
	}
	return 0;
}
#endif

static int m95p_init(const struct device *dev)
{
	const struct m95p_config *cfg = dev->config;
	int err;

	if (!spi_is_ready_dt(&cfg->spi)) {
		LOG_ERR("SPI bus not ready");
		return -ENODEV;
	}

#if ANY_INST_HAS_WP_GPIOS
	if (cfg->wp) {
		if (!gpio_is_ready_dt(cfg->wp)) {
			LOG_ERR("WP GPIO not ready");
			return -ENODEV;
		}
		/* Start write-protected (active-low → OUTPUT_ACTIVE = pin LOW). */
		err = gpio_pin_configure_dt(cfg->wp, GPIO_OUTPUT_ACTIVE);
		if (err != 0) {
			LOG_ERR("WP GPIO configure failed: %d", err);
			return err;
		}
	}
#endif

	/*
	 * Guarantee a CS falling edge after power-up (DS13999 power-up sequence).
	 * The device ignores all commands until it sees CS go low after Vcc is
	 * valid.  Sending 0xAB (RDP) provides that edge and also wakes the device
	 * if a previous session left it in deep power-down mode.
	 */
	m95p_cmd1(dev, M95P_OP_RDP);
	k_msleep(1);

	err = m95p_wait_ready(dev, 10);
	if (err != 0) {
		LOG_ERR("Device not ready after power-up");
		return err;
	}

#if !defined(CONFIG_FLASH_M95PXX_SKIP_JEDEC_CHECK)
	err = m95p_check_jedec(dev);
	if (err != 0) {
		return err;
	}
#else
	LOG_WRN("JEDEC ID check skipped");
#endif

	LOG_INF("M95P ready: %u KB", cfg->chip_size / 1024);
	return 0;
}

static DEVICE_API(flash, m95p_api) = {
	.read = m95p_read,
	.write = m95p_write,
	.erase = m95p_erase,
	.get_parameters = m95p_get_parameters,
	.get_size = m95p_get_size,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.page_layout = m95p_pages_layout,
#endif
};

#define INST_HAS_WP(n) DT_INST_NODE_HAS_PROP(n, wp_gpios)

#define M95P_WP_SPEC(n)                                                                            \
	IF_ENABLED(INST_HAS_WP(n), (                                     \
		static const struct gpio_dt_spec m95p_wp_##n =           \
			GPIO_DT_SPEC_INST_GET(n, wp_gpios);))

#define M95P_INST(n)                                                                               \
	M95P_WP_SPEC(n)                                                                            \
	static struct m95p_data m95p_data_##n = {                                                  \
		.lock = Z_SEM_INITIALIZER(m95p_data_##n.lock, 1, 1),                               \
	};                                                                                         \
	static const struct m95p_config m95p_cfg_##n = {                                           \
		.spi = SPI_DT_SPEC_INST_GET(n, SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB |             \
						       SPI_WORD_SET(8)),                           \
		.chip_size = DT_INST_PROP(n, size),                                                \
		.jedec_id = DT_INST_PROP(n, jedec_id),                                             \
		IF_ENABLED(INST_HAS_WP(n), (.wp = &m95p_wp_##n,))                                  \
				IF_ENABLED(CONFIG_FLASH_PAGE_LAYOUT, (                           \
			.layout = {                                              \
				.pages_count = DT_INST_PROP(n, size) /           \
					       M95P_SECTOR_SIZE,                 \
				.pages_size  = M95P_SECTOR_SIZE,                 \
			},)) };  \
	DEVICE_DT_INST_DEFINE(n, m95p_init, NULL, &m95p_data_##n, &m95p_cfg_##n, POST_KERNEL,      \
			      CONFIG_FLASH_M95PXX_INIT_PRIORITY, &m95p_api);

DT_INST_FOREACH_STATUS_OKAY(M95P_INST)
