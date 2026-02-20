#include "vmt_flash.h"

/* Private includes ----------------------------------------------------------*/
#include <string.h>
#include <stdio.h>

#include "stm32f7xx_hal_flash_ex.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define FLASH_TIMEOUT_VALUE 50000U /* 50 s */

/* Private function prototypes -----------------------------------------------*/
/**
 * @brief  Set the specific FLASH error flag.
 * @retval None
 */
static void FLASH_SetErrorCode(void);

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
extern FLASH_ProcessTypeDef pFlash;

static size_t        program_size    = 0;
static uint8_t      *p_program_data  = NULL;
static __IO uint8_t *p_program_flash = NULL;

// static uint32_t flash_opr_addr = 0;

/* Private user code ---------------------------------------------------------*/
void HAL_FLASH_EndOfOperationCallback(uint32_t ReturnValue)
{
    flash_cplt_callback(pFlash.ProcedureOnGoing, ReturnValue);

    // HAL_FLASH_Lock();

    // if (ReturnValue == 0xFFFFFFFF)
    // {
    //     flash_ops_cplt_callback(FLASH_OPS_SECTORS_ERASE);
    // }
    // else if (ReturnValue == flash_opr_addr)
    // {
    //     flash_ops_cplt_callback(FLASH_OPS_PROGRAM);
    //     flash_opr_addr = 0;
    // }
    // else
    // {
    //     flash_ops_cplt_callback(FLASH_OPS_MASS_ERASE);
    // }
}

void HAL_FLASH_OperationErrorCallback(uint32_t ReturnValue)
{
    flash_cplt_callback(pFlash.ProcedureOnGoing, ReturnValue);

    // if (ReturnValue == 0xFFFFFFFF)
    // {
    //     flash_ops_error_callback(FLASH_OPS_SECTORS_ERASE);
    // }
    // else if (ReturnValue == flash_opr_addr)
    // {
    //     flash_ops_error_callback(FLASH_OPS_PROGRAM);
    //     flash_opr_addr = 0;
    // }
    // else
    // {
    //     flash_ops_error_callback(FLASH_OPS_MASS_ERASE);
    // }
}

bool flash_process(void)
{
    typedef enum step_
    {
        STEP_IDLE,
        STEP_BUSY_WAIT,
        STEP_PROGRAM_START,
        STEP_PROGRAM_RUN,
        STEP_PROGRAM_END,
        STEP_FINISH,
    } step_t;
    static step_t step = STEP_IDLE;
    static size_t byte_n;

    switch (step)
    {
    case STEP_FINISH:
        step = STEP_IDLE;
    case STEP_IDLE:
        if (p_program_data == NULL)
            break;
        step = STEP_BUSY_WAIT;
    case STEP_BUSY_WAIT:
        if (pFlash.Lock == HAL_LOCKED)
            break;
        if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY))
            break;
        if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_ALL_ERRORS) != RESET)
        {
            /*Save the error code*/
            FLASH_SetErrorCode();
            break;
        }
        step = STEP_PROGRAM_START;
    case STEP_PROGRAM_START:
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP);
        HAL_FLASH_Unlock();
        /* If the previous operation is completed, proceed to program the new data */
        FLASH->CR &= CR_PSIZE_MASK;
        FLASH->CR |= FLASH_PSIZE_BYTE;
        FLASH->CR |= FLASH_CR_PG;

        byte_n = 0;
        step   = STEP_PROGRAM_RUN;
    case STEP_PROGRAM_RUN:
        if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY))
            break;
        for (; byte_n < program_size; byte_n++)
        {
            p_program_flash[byte_n] = p_program_data[byte_n];
            __DSB();
        }
        // if (byte_n < program_size)
        // {
        //     p_program_flash[byte_n] = p_program_data[byte_n];
        //     /* Data synchronous Barrier (DSB) Just after the write operation
        //      This will force the CPU to respect the sequence of instruction (no opsimization).*/
        //     __DSB();

        //     byte_n++;
        //     break;
        // }
        step = STEP_PROGRAM_END;
        // break;
    case STEP_PROGRAM_END:
        FLASH->CR &= (~FLASH_CR_PG);
        HAL_FLASH_Lock();
        __HAL_UNLOCK(&pFlash);

        if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_ALL_ERRORS) != RESET)
        {
            /*Save the error code*/
            FLASH_SetErrorCode();
        }
        p_program_data = NULL;

        flash_cplt_callback(FLASH_PROC_PROGRAM, (size_t)p_program_flash);
        step = STEP_FINISH;
    }
    return step != STEP_IDLE;
}

flash_res_t flash_program_data(uint32_t address, void *p_data, size_t data_size)
{
    if (p_program_data != NULL)
        return FLASH_RES_BUSY;
    if (p_data == NULL)
        return FLASH_RES_ERROR;
    if (data_size == 0)
        return FLASH_RES_ERROR;
    if (IS_FLASH_ADDRESS(address) == false)
        return FLASH_RES_ERROR;

    p_program_data  = p_data;
    program_size    = data_size;
    p_program_flash = (void *)address;
    return FLASH_RES_OK;
}
// bool flash_program_data(uint32_t address, void *p_data, size_t data_size)
// {
//     if (p_data == NULL)
//         return false;

//     if (data_size == 0)
//         return false;

//     if (IS_FLASH_ADDRESS(address) == false)
//         return false;

//     HAL_StatusTypeDef status = HAL_ERROR;

//     HAL_FLASH_Unlock();

//     /* Process Locked */
//     __HAL_LOCK(&pFlash);

//     /* Wait for last operation to be completed */
//     status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);

//     if (status == HAL_OK)
//     {
//         /* Check the parameters */
//         assert_param(IS_FLASH_ADDRESS(address));

//         //        flash_opr_addr = address;

//         /* If the previous operation is completed, proceed to program the new data */
//         FLASH->CR &= CR_PSIZE_MASK;
//         FLASH->CR |= FLASH_PSIZE_BYTE;
//         FLASH->CR |= FLASH_CR_PG;

//         /* Program the byte */
//         __IO uint8_t *p_program = (void *)address;
//         uint8_t      *p_buff    = p_data;
//         //        uint8_t check_sum = 0;
//         for (size_t i = 0; i < data_size; i++)
//         {
//             p_program[i] = p_buff[i];
//             /* Data synchronous Barrier (DSB) Just after the write operation
//              This will force the CPU to respect the sequence of instruction (no opsimization).*/
//             __DSB();

//             //            check_sum += p_buff[i];
//         }

//         //        /* Program the check_sum */
//         //        p_program[data_size] = check_sum;
//         //        __DSB();

//         /* Wait for last operation to be completed */
//         status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);

//         /* If the program operation is completed, disable the PG Bit */
//         FLASH->CR &= (~FLASH_CR_PG);
//     }

//     /* Process Unlocked */
//     __HAL_UNLOCK(&pFlash);

//     HAL_FLASH_Lock();

//     return status == HAL_OK;
// }

bool flash_program_busy_get(void)
{
    return p_program_data != NULL;
}

bool flash_read_data(uint32_t address, void *p_data, size_t data_size)
{
    if (p_data == NULL)
        return false;

    if (data_size == 0)
        return false;

    memcpy(p_data, (uint32_t *)address, data_size);
    return true;
}

/* @param  sector : FLASH sector to erase.
 *         The value of this parameter depend on device used within the same series.
 *         This parameter must be a value of @ref FLASHEx_Sectors.
 *   @arg FLASH_SECTOR_0 : 0U; Sector Number 0, 0x0800 0000 - 0x0800 3FFF, 16KB
 *   @arg FLASH_SECTOR_1 : 1U; Sector Number 1, 0x0800 4000 - 0x0800 7FFF, 16KB
 *   @arg FLASH_SECTOR_2 : 2U; Sector Number 2, 0x0800 8000 - 0x0800 BFFF, 16KB
 *   @arg FLASH_SECTOR_3 : 3U; Sector Number 3, 0x0800 C000 - 0x0800 FFFF, 16KB
 *   @arg FLASH_SECTOR_4 : 4U; Sector Number 4, 0x0801 0000 - 0x0801 FFFF, 64KB
 *   @arg FLASH_SECTOR_5 : 5U; Sector Number 5, 0x0802 0000 - 0x0803 FFFF, 128KB
 *   @arg FLASH_SECTOR_6 : 6U; Sector Number 6, 0x0804 0000 - 0x0805 FFFF, 128KB
 *   @arg FLASH_SECTOR_7 : 7U; Sector Number 7, 0x0806 0000 - 0x0807 FFFF, 128KB
 * @retval :
 *   FLASH_RES_OK : erase success.
 *   FLASH_RES_BUSY : flash busy.
 *   FLASH_RES_ERROR : flash error, error code can get from flash_error_get() function.
 **/
flash_res_t flash_erase_sector(uint32_t sector)
{
    if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY))
        return FLASH_RES_BUSY;

    if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_ALL_ERRORS) != RESET)
    {
        /*Save the error code*/
        FLASH_SetErrorCode();
        return FLASH_RES_ERROR;
    }

    HAL_FLASH_Unlock();

    /* Process Locked */
    if (pFlash.Lock == HAL_LOCKED)
    {
        return FLASH_RES_BUSY;
    }
    else
    {
        pFlash.Lock = HAL_LOCKED;
    }

    /* Enable End of FLASH Operation interrupt */
    __HAL_FLASH_ENABLE_IT(FLASH_IT_EOP);

    /* Enable Error source interrupt */
    __HAL_FLASH_ENABLE_IT(FLASH_IT_ERR);

    /* Clear pending flags (if any) */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_ERSERR);

    /* Erase by sector to be done*/

    /* Check the parameters */
    assert_param(IS_FLASH_NBSECTORS(h_flash_erase_init.NbSectors + sector));

    pFlash.ProcedureOnGoing = FLASH_PROC_SECTERASE;
    pFlash.NbSectorsToErase = 1;
    pFlash.Sector           = sector;
    pFlash.VoltageForErase  = FLASH_VOLTAGE_RANGE_3;

    /*Erase 1st sector and wait for IT*/
    FLASH_Erase_Sector(sector, FLASH_VOLTAGE_RANGE_3);

    __HAL_UNLOCK(&pFlash);
    return FLASH_RES_OK;
}

/* return value :
 *   HAL_FLASH_ERROR_NONE      : 0x00000000U No error
 *   HAL_FLASH_ERROR_ERS       : 0x00000002U Programming Sequence error
 *   HAL_FLASH_ERROR_PGP       : 0x00000004U Programming Parallelism error
 *   HAL_FLASH_ERROR_PGA       : 0x00000008U Programming Alignment error
 *   HAL_FLASH_ERROR_WRP       : 0x00000010U Write protection error
 *   HAL_FLASH_ERROR_OPERATION : 0x00000020U Operation Error
 *   HAL_FLASH_ERROR_RD        : 0x00000040U Read Protection Error
 * */
uint32_t flash_error_get(void)
{
    return pFlash.ErrorCode;
}
void flash_error_clear(void)
{
    pFlash.ErrorCode = HAL_FLASH_ERROR_NONE;
}

__weak void flash_cplt_callback(FLASH_ProcedureTypeDef proc, uint32_t ReturnValue)
{
}
__weak void flash_ops_error_callback(FLASH_ProcedureTypeDef proc, uint32_t ReturnValue)
{
}

/**
 * @brief  Set the specific FLASH error flag.
 * @retval None
 */
static void FLASH_SetErrorCode(void)
{
    if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_OPERR) != RESET)
    {
        pFlash.ErrorCode |= HAL_FLASH_ERROR_OPERATION;
    }

    if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_WRPERR) != RESET)
    {
        pFlash.ErrorCode |= HAL_FLASH_ERROR_WRP;
    }

    if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_PGAERR) != RESET)
    {
        pFlash.ErrorCode |= HAL_FLASH_ERROR_PGA;
    }

    if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_PGPERR) != RESET)
    {
        pFlash.ErrorCode |= HAL_FLASH_ERROR_PGP;
    }

    if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_ERSERR) != RESET)
    {
        pFlash.ErrorCode |= HAL_FLASH_ERROR_ERS;
    }

#if defined(FLASH_OPTCR2_PCROP)
    if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_RDERR) != RESET)
    {
        pFlash.ErrorCode |= HAL_FLASH_ERROR_RD;
    }
#endif /* FLASH_OPTCR2_PCROP */

    /* Clear error programming flags */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
}
