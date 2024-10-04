/*
 * littlefs_driver.h
 *
 *  Created on: Sep 29, 2024
 *      Author: Sangam
 */

#ifndef INC_LITTLEFS_DRIVER_H_
#define INC_LITTLEFS_DRIVER_H_
#include "main.h"
#include "lfs_util.h"
#include "lfs.h"
#include "nor.h"

extern UART_HandleTypeDef huart1;
/*Definition for debug uart, spi port and gpio pin*/
#define DEBUG_UART huart1
#define CS_FLASH_MEMORY_PORT GPIOB
#define CS_FLASH_MEMORY_PIN GPIO_PIN_12
#define SPI_PIN_FLASH hspi2

extern SPI_HandleTypeDef hspi2;
#define PATH_MAX_LEN 256
lfs_file_t File;
char Text[20];
lfs_t Lfs,Lfs2;
nor_t Nor;
static struct lfs_config LfsConfig,LfsConfig2 = {0};

void read_file_from_littlefs(lfs_t *lfs, const char *filename) ;
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);
void nor_delay_us(uint32_t us);
void nor_cs_assert();
void nor_cs_deassert();
void nor_spi_tx(uint8_t *pData, uint32_t Size);
void nor_spi_rx(uint8_t *pData, uint32_t Size);
void __init_nor();
int _fs_read(const struct lfs_config *c, lfs_block_t block,
            lfs_off_t off, void *buffer, lfs_size_t size);
int _fs_write(const struct lfs_config *c, lfs_block_t block,
        lfs_off_t off, const void *buffer, lfs_size_t size);
int _fs_erase(const struct lfs_config *c, lfs_block_t block);
int _fs_sync(const struct lfs_config *c);
void list_files(lfs_t *lfs) ;
void __init_littefs();
int count_files_in_directory(lfs_t *lfs, const char *path);
void list_directories_with_file_count(lfs_t *lfs, const char *path);
void list_files_with_size(lfs_t *lfs, const char *path);
void __init_storage();
#endif /* INC_LITTLEFS_DRIVER_H_ */
