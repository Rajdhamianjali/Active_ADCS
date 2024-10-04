/*
 * littlefs_driver.c
 *
 *  Created on: Sep 29, 2024
 *      Author: Sangam
 */
#include "littlefs_driver.h"




void read_file_from_littlefs(lfs_t *lfs, const char *filename) {
    lfs_file_t file;
    HAL_UART_Transmit(&DEBUG_UART, filename,sizeof(filename),1000);
    // Open the file for reading
    int err = lfs_file_open(lfs, &file, filename, LFS_O_RDONLY);
    if (err < 0) {
        printf("Failed to open file: %s\n", filename);
        return;
    }

    // Get the file size
    lfs_soff_t file_size = lfs_file_size(lfs, &file);
    if (file_size < 0) {
        printf("Failed to get file size for: %s\n", filename);
        lfs_file_close(lfs, &file);
        return;
    }

    // Allocate a buffer to hold the file data
    float *buffer = malloc(file_size);
    if (buffer == NULL) {
        printf("Failed to allocate buffer for reading file: %s\n", filename);
        lfs_file_close(lfs, &file);
        return;
    }

    // Read the file content into the buffer
    lfs_ssize_t bytes_read = lfs_file_read(lfs, &file, buffer, file_size);
    if (bytes_read < 0) {
        printf("Failed to read file: %s\n", filename);
    } else {
    	char x;
        // Successfully read the file, print its content (if it's text data)
        HAL_UART_Transmit(&DEBUG_UART, buffer, (int)bytes_read,1000);
//        for(int i=0;i<(int) bytes_read;){
////        	printf(buffer[i]);
//         	x=buffer[i];
//        	i++;
//        }


        printf("File Content (%s):\n%.*s\n", filename, (int)bytes_read, buffer);
    }

    // Clean up
    free(buffer);
    lfs_file_close(lfs, &file);
}

volatile uint8_t DmaEnd = 0;

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){
	DmaEnd = 1;
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi){
	DmaEnd = 1;
}

void nor_delay_us(uint32_t us){
	HAL_Delay(1000);
}

void nor_cs_assert(){
	HAL_GPIO_WritePin(CS_FLASH_MEMORY_PORT, CS_FLASH_MEMORY_PIN, GPIO_PIN_RESET);
}

void nor_cs_deassert(){
	HAL_GPIO_WritePin(CS_FLASH_MEMORY_PORT, CS_FLASH_MEMORY_PIN, GPIO_PIN_SET);
}

void nor_spi_tx(uint8_t *pData, uint32_t Size){
	DmaEnd = 0;
	HAL_SPI_Transmit(&SPI_PIN_FLASH, pData, Size, 1000);

}

void nor_spi_rx(uint8_t *pData, uint32_t Size){
	DmaEnd = 0;
	HAL_SPI_Receive(&SPI_PIN_FLASH, pData, Size, 1000);

}

void __init_nor(){
	Nor.config.CsAssert = nor_cs_assert;
	Nor.config.CsDeassert = nor_cs_deassert;
	Nor.config.DelayUs = nor_delay_us;
	Nor.config.SpiRxFxn = nor_spi_rx;
	Nor.config.SpiTxFxn = nor_spi_tx;

	if (NOR_Init(&Nor) != NOR_OK){ //NOR_Init
		Error_Handler();
	}
}

/** Start LittleFs **/

int _fs_read(const struct lfs_config *c, lfs_block_t block,
            lfs_off_t off, void *buffer, lfs_size_t size){

	if (NOR_ReadSector(&Nor, (uint8_t*)buffer, block, off, size) == NOR_OK){
		return 0;
	}

	return LFS_ERR_IO;
}

int _fs_write(const struct lfs_config *c, lfs_block_t block,
        lfs_off_t off, const void *buffer, lfs_size_t size){

	if (NOR_WriteSector(&Nor, (uint8_t*)buffer, block, off, size) == NOR_OK){
		return 0;
	}

	return LFS_ERR_IO;
}

int _fs_erase(const struct lfs_config *c, lfs_block_t block){
	if (NOR_EraseSector(&Nor, block) == NOR_OK){
		return 0;
	}

	return LFS_ERR_IO;
}

int _fs_sync(const struct lfs_config *c){
	return 0;
}

// Function to list all files and directories in the filesystem
void list_files(lfs_t *lfs) {
    lfs_dir_t dir;
    struct lfs_info info;

    // Open the root directory
    int err = lfs_dir_open(lfs, &dir, "/");
    if (err) {
        printf("Failed to open directory\n");
        return;
    }

    // Loop through all files in the directory
    while (true) {
        err = lfs_dir_read(lfs, &dir, &info);
        if (err < 0) {
            printf("Failed to read directory\n");
            break;
        }

        // If no more files, break
        if (err == 0) {
            break;
        }
        uint8_t dir[100];
        // Print the type and name of the file
        if (info.type == LFS_TYPE_REG) {
            sprintf(dir,"File: %s\n\0", info.name);
            HAL_UART_Transmit(&DEBUG_UART, dir, strlen(dir),1000);
        } else if (info.type == LFS_TYPE_DIR) {
        	sprintf(dir,"Directory: %s\n\0", info.name);

            HAL_UART_Transmit(&DEBUG_UART, dir, strlen(dir),1000);
        }
    }

    // Close the directory
    lfs_dir_close(lfs, &dir);
}
void __init_littefs(){
	// because of static qualifier, this variable
	// will have a dedicated address
		int Error;

		LfsConfig.read_size = 256;
		LfsConfig.prog_size = 256;
		LfsConfig.block_size = Nor.info.u16SectorSize;
		LfsConfig.block_count =  16384;//Nor.info.u32SectorCount;
		LfsConfig.cache_size = Nor.info.u16PageSize;
		LfsConfig.lookahead_size = 15000;//Nor.info.u32SectorCount/8;
		LfsConfig.block_cycles = 100;
		LfsConfig.context = (void*) (40* 16384 * Nor.info.u16SectorSize);

		LfsConfig.read = _fs_read;
		LfsConfig.prog = _fs_write;
		LfsConfig.erase = _fs_erase;
		LfsConfig.sync = _fs_sync;

		Error = lfs_mount(&Lfs, &LfsConfig);
		if (Error != LFS_ERR_OK){
			lfs_format(&Lfs, &LfsConfig);
			Error = lfs_mount(&Lfs, &LfsConfig);
			if (Error != LFS_ERR_OK){
				Error_Handler();
			}
		}


		LfsConfig2.read_size = 256;
		LfsConfig2.prog_size = 256;
		LfsConfig2.block_size = Nor.info.u16SectorSize;
		LfsConfig2.block_count =  16384;//Nor.info.u32SectorCount;
		LfsConfig2.cache_size = Nor.info.u16PageSize;//1024
		LfsConfig2.lookahead_size = 5256;//Nor.info.u32SectorCount/8;
		LfsConfig2.block_cycles = 100;

		LfsConfig2.read = _fs_read;
		LfsConfig2.prog = _fs_write;
		LfsConfig2.erase = _fs_erase;
		LfsConfig2.sync = _fs_sync;

		LfsConfig2.context = (void*) (4* 16384 * Nor.info.u16SectorSize);
//		Error = lfs_mount(&Lfs2, &LfsConfig2);
//				if (Error != LFS_ERR_OK){
//					lfs_format(&Lfs2, &LfsConfig2);
//					Error = lfs_mount(&Lfs2, &LfsConfig2);
//					if (Error != LFS_ERR_OK){
//						Error_Handler();
//					}
//				}

}


// Function to count the number of files in a directory
int count_files_in_directory(lfs_t *lfs, const char *path) {
    lfs_dir_t dir;
    struct lfs_info info;
    int file_count = 0;

    // Open the directory at the given path
    int err = lfs_dir_open(lfs, &dir, path);
    if (err) {
        printf("Failed to open directory: %s\n", path);
        return -1;
    }

    // Loop through all files in the directory
    while (true) {
        err = lfs_dir_read(lfs, &dir, &info);
        if (err < 0) {
            printf("Failed to read directory: %s\n", path);
            break;
        }

        // If no more files, break
        if (err == 0) {
            break;
        }

        // Check if the entry is a file
        if (info.type == LFS_TYPE_REG) {
            file_count++;
        }
    }

    // Close the directory
    lfs_dir_close(lfs, &dir);

    return file_count;
}
// Function to list directories and their contents
void list_directories_with_file_count(lfs_t *lfs, const char *path) {
    lfs_dir_t dir;
    struct lfs_info info;

    // Open the directory at the given path
    int err = lfs_dir_open(lfs, &dir, path);
    if (err) {
        printf("Failed to open directory: %s\n", path);
        return;
    }

    // Loop through all entries in the directory
    while (true) {
        err = lfs_dir_read(lfs, &dir, &info);
        if (err < 0) {
            printf("Failed to read directory: %s\n", path);
            break;
        }

        // If no more entries, break
        if (err == 0) {
            break;
        }

        // Build the full path for the current file/directory
        char full_path[PATH_MAX_LEN];
        snprintf(full_path, sizeof(full_path), "%s/%s", path, info.name);

        // Check if the entry is a directory (excluding "." and "..")
        if (info.type == LFS_TYPE_DIR && strcmp(info.name, ".") != 0 && strcmp(info.name, "..") != 0) {
            int file_count = count_files_in_directory(lfs, full_path);
            char pa[500];
            sprintf(pa, "Directory: %s, Number of files: %d\n", full_path, file_count);
            HAL_UART_Transmit(&DEBUG_UART, (uint8_t *)pa, strlen(pa), 1000);

            // Recursively list the contents of the directory
            list_directories_with_file_count(lfs, full_path);
        }
    }

    // Close the directory
    lfs_dir_close(lfs, &dir);
}


// Recursive function to list files and directories with full paths
void list_files_with_size(lfs_t *lfs, const char *path) {
    lfs_dir_t dir;
    struct lfs_info info;

    // Open the directory at the given path
    int err = lfs_dir_open(lfs, &dir, path);
    if (err) {
        printf("Failed to open directory: %s\n", path);
        return;
    }

    // Loop through all files in the directory
    while (true) {
        err = lfs_dir_read(lfs, &dir, &info);
        if (err < 0) {
            printf("Failed to read directory: %s\n", path);
            break;
        }

        // If no more files, break
        if (err == 0) {
            break;
        }

        // Build the full path for the current file/directory
        char full_path[PATH_MAX_LEN];
        snprintf(full_path, sizeof(full_path), "%s/%s", path, info.name);
        HAL_UART_Transmit(&DEBUG_UART, info.type, strlen(info.type),1000);
        char pa[500];
        // Check if the entry is a file or directory
        if (info.type == LFS_TYPE_REG) {
            sprintf(pa,"File: %s, Size: %ld bytes\n", full_path, info.size);
            HAL_UART_Transmit(&DEBUG_UART, pa, strlen(pa),1000);
        } else if (info.type == LFS_TYPE_DIR && strcmp(info.name, ".") != 0 && strcmp(info.name, "..") != 0) {
            sprintf(pa,"Directory: %s\n", full_path);
            HAL_UART_Transmit(&DEBUG_UART, pa, strlen(pa),1000);
            // Recursively list the contents of the directory
            list_files_with_size(lfs, full_path);
        }
    }

    // Close the directory
    lfs_dir_close(lfs, &dir);
}
void __init_storage(){
	__init_nor();
	__init_littefs();
}

void write_to_file(char *filename, uint8_t *data){
	  __init_storage();
	  list_directories_with_file_count(&Lfs,"");
	  lfs_file_open(&Lfs, &File, filename, LFS_O_CREAT | LFS_O_RDWR  |LFS_O_APPEND );
	  lfs_file_write(&Lfs, &File, data, sizeof(data));
	  lfs_file_close(&Lfs, &File);
	  lfs_unmount(&Lfs);
}

