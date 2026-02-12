#include "stdio.h"
#include "stdint.h"
#include "stdbool.h"
#include "string.h"
#include "math.h"

#include "parameters.h"
#include "lfs_api.h"

int w25nxx_lfs_read(const struct lfs_config *c, lfs_block_t block, 
                        lfs_off_t off, void *buffer, lfs_size_t size)
{
    int err_code = 0;

    err_code = w25nxx_block_read_with_BUF(&g_w25nxx_dev, block, off, (uint8_t*) buffer, size);

    if(err_code == 0)
    {
        return  LFS_ERR_OK;
    }

    return  LFS_ERR_INVAL;
}

int w25nxx_lfs_write(const struct lfs_config *c, lfs_block_t block,
				        lfs_off_t off,void *buffer, lfs_size_t size)
{
    int err_code = 0;

    err_code = w25nxx_block_write(&g_w25nxx_dev, block, off, (uint8_t*) buffer, size);

    if(err_code == 0)
    {
        return  LFS_ERR_OK;
    }

    return  LFS_ERR_INVAL;
}

int w25nxx_lfs_erase(const struct lfs_config *c, lfs_block_t block)
{
	int err_code = 0;

    err_code = w25nxx_block_erase(&g_w25nxx_dev, block);

    if(err_code == 0)
    {
        return  LFS_ERR_OK;
    }

    return  LFS_ERR_INVAL;
}

int w25nxx_lfs_sync(const struct lfs_config *c )
{
	return  LFS_ERR_OK;
}
