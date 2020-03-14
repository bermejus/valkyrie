#include "xdmac.hpp"

/*
    * Configure DMA for a transfer.
    *
    * \param[out] xdmac Module hardware register base address pointer.
    * \param[in] channel_num The used channel number.
    * \param[in] cfg   The configuration for used channel
    */
void xdmac_configure_transfer(Xdmac* xdmac, uint32_t channel_num, xdmac_channel_config_t* cfg)
{
    xdmac_channel_get_interrupt_status(xdmac, channel_num);
    xdmac_channel_set_source_addr(xdmac, channel_num, cfg->mbr_sa);
    xdmac_channel_set_destination_addr(xdmac, channel_num, cfg->mbr_da);
    xdmac_channel_set_microblock_control(xdmac, channel_num, cfg->mbr_ubc);
    xdmac_channel_set_block_control(xdmac, channel_num, cfg->mbr_bc);
    xdmac_channel_set_datastride_mempattern(xdmac, channel_num, cfg->mbr_ds);
    xdmac_channel_set_source_microblock_stride(xdmac, channel_num, cfg->mbr_sus);
    xdmac_channel_set_destination_microblock_stride(xdmac, channel_num, cfg->mbr_dus);
    xdmac_channel_set_config(xdmac, channel_num, cfg->mbr_cfg);
}