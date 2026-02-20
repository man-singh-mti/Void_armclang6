#ifndef __vmt_flash_H
#define __vmt_flash_H

/* includes ----------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

#include "main.h"

/* type define -------------------------------------------------------*/
typedef enum flash_res_
{
    FLASH_RES_ERROR,
    FLASH_RES_OK,
    FLASH_RES_BUSY,
} flash_res_t;

/* function prototypes -----------------------------------------------*/
bool flash_process(void);

flash_res_t flash_program_data(uint32_t address, void *p_data, size_t data_size);
bool        flash_program_busy_get(void);

bool flash_read_data(uint32_t address, void *p_data, size_t data_size);

/* @param sector : FLASH sector to erase.
 *        The value of this parameter depend on device used within the same series.
 *        This parameter must be a value of @ref FLASHEx_Sectors.
 *   @arg FLASH_SECTOR_0 : 0U; Sector Number 0, 0x0800 0000 - 0x0800 3FFF, 16KB
 *   @arg FLASH_SECTOR_1 : 1U; Sector Number 1, 0x0800 4000 - 0x0800 7FFF, 16KB
 *   @arg FLASH_SECTOR_2 : 2U; Sector Number 2, 0x0800 8000 - 0x0800 BFFF, 16KB
 *   @arg FLASH_SECTOR_3 : 3U; Sector Number 3, 0x0800 C000 - 0x0800 FFFF, 16KB
 *   @arg FLASH_SECTOR_4 : 4U; Sector Number 4, 0x0801 0000 - 0x0801 FFFF, 64KB
 *   @arg FLASH_SECTOR_5 : 5U; Sector Number 5, 0x0802 0000 - 0x0803 FFFF, 128KB
 *   @arg FLASH_SECTOR_6 : 6U; Sector Number 6, 0x0804 0000 - 0x0805 FFFF, 128KB
 *   @arg FLASH_SECTOR_7 : 7U; Sector Number 7, 0x0806 0000 - 0x0807 FFFF, 128KB
 * @retval :
 *   FLASH_RES_OK    : erase success.
 *   FLASH_RES_BUSY  : flash busy.
 *   FLASH_RES_ERROR : flash error, error code can get from flash_error_get() function.
 */
flash_res_t flash_erase_sector(uint32_t sector);

/* return value :
 *   HAL_FLASH_ERROR_NONE      : 0x00000000U No error
 *   HAL_FLASH_ERROR_ERS       : 0x00000002U Programming Sequence error
 *   HAL_FLASH_ERROR_PGP       : 0x00000004U Programming Parallelism error
 *   HAL_FLASH_ERROR_PGA       : 0x00000008U Programming Alignment error
 *   HAL_FLASH_ERROR_WRP       : 0x00000010U Write protection error
 *   HAL_FLASH_ERROR_OPERATION : 0x00000020U Operation Error
 *   HAL_FLASH_ERROR_RD        : 0x00000040U Read Protection Error
 * */
uint32_t flash_error_get(void);
void     flash_error_clear(void);

/* ReturnValue :
 *   if proc is FLASH_PROC_SECTERASE, value is Sector of erase.
 *     value is 0xFFFFFFFFU, when erase finish.
 *   if proc is FLASH_PROC_MASSERASE, value is 0.
 *   if proc is FLASH_PROC_PROGRAM, value is Address of program.
 * */
void flash_cplt_callback(FLASH_ProcedureTypeDef proc, uint32_t ReturnValue);
void flash_error_callback(FLASH_ProcedureTypeDef proc, uint32_t ReturnValue);

#endif /* __vmt_flash_H */
