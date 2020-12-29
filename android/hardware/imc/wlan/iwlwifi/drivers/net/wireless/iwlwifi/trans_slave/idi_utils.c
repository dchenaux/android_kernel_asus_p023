/******************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2013 - 2014 Intel Corporation. All rights reserved.
 * Copyright(c) 2013 - 2014 Intel Mobile Communications GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110,
 * USA
 *
 * The full GNU General Public License is included in this distribution
 * in the file called COPYING.
 *
 * Contact Information:
 *  Intel Linux Wireless <ilw@linux.intel.com>
 * Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497
 *
 * BSD LICENSE
 *
 * Copyright(c) 2012 - 2014 Intel Corporation. All rights reserved.
 * Copyright(c) 2013 - 2014 Intel Mobile Communications GmbH
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name Intel Corporation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

#include "iwl-debug.h"
#include "idi_internal.h"
#include "idi_utils.h"

int iwl_idi_alloc_dma_mem(struct iwl_trans *trans,
			  struct iwl_idi_dma_ptr *ptr,
			  size_t size,
			  enum dma_data_direction dir)
{
#ifdef CONFIG_X86
	ptr->addr = kmalloc(size, GFP_KERNEL);
	if (!ptr->addr)
		goto err;

	ptr->dma = dma_map_single(trans->dev, ptr->addr, size, dir);
	if (unlikely(dma_mapping_error(trans->dev, ptr->dma))) {
		kfree(ptr->addr);
		goto err;
	}
#else
	ptr->addr = dma_alloc_coherent(trans->dev, size, &ptr->dma, GFP_KERNEL);
	if (!ptr->addr)
		goto err;
#endif

	ptr->size = size;
	ptr->mapping_dir = dir;
	return 0;

err:
	ptr->addr = NULL;
	ptr->size = 0;
	return -ENOMEM;
}

void iwl_idi_free_dma_mem(struct iwl_trans *trans,
			  struct iwl_idi_dma_ptr *ptr)
{
	if (!ptr->size)
		return;

#ifdef CONFIG_X86
	dma_unmap_single(trans->dev, ptr->dma, ptr->size, ptr->mapping_dir);
	kfree(ptr->addr);
#else
	dma_free_coherent(trans->dev, ptr->size, ptr->addr, ptr->dma);
#endif
	ptr->addr = NULL;
	ptr->size = 0;
}

int iwl_idi_alloc_sg_list(struct iwl_trans *trans,
			  struct iwl_idi_dma_ptr *sg_list,
			  size_t size,
			  enum dma_data_direction dir)
{
	int ret;
	dma_addr_t dma_aligned;

	/* add 16 to have a room for alinment shift */
	size += 16;

	ret = iwl_idi_alloc_dma_mem(trans, sg_list, size, dir);
	if (ret)
		return ret;

	memset(sg_list->addr, 0, size);

	/* S/G list address alignment is to 16 bytes */
	dma_aligned = ALIGN(sg_list->dma, 16);
	sg_list->align_offset = dma_aligned - sg_list->dma;
	sg_list->dma += sg_list->align_offset;
	sg_list->addr = (u8 *)sg_list->addr + sg_list->align_offset;

	return 0;
}

void iwl_idi_free_sg_list(struct iwl_trans *trans,
			  struct iwl_idi_dma_ptr *ptr)
{
	/*
	 * The addresses where adjusted to align_offset after the allocation.
	 * Need to move it back before freeing.
	 */
	ptr->addr = (u8 *)ptr->addr - ptr->align_offset;
	ptr->dma = ptr->dma - ptr->align_offset;

	iwl_idi_free_dma_mem(trans, ptr);
}
