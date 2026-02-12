#ifndef LFS_API_H__
#define LFS_API_H__

#include "lfs.h"
#include "lfs_util.h"

int w25nxx_lfs_read(const struct lfs_config *c, lfs_block_t block, 
                        lfs_off_t off, void *buffer, lfs_size_t size);

int w25nxx_lfs_write(const struct lfs_config *c, lfs_block_t block,
				        lfs_off_t off,void *buffer, lfs_size_t size);

int w25nxx_lfs_erase(const struct lfs_config *c, lfs_block_t block);

int w25nxx_lfs_sync(const struct lfs_config *c );

#endif
