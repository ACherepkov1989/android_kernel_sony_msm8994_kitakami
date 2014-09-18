/**
 * host.c - DesignWare USB3 DRD Controller Host Glue
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com
 *
 * Authors: Felipe Balbi <balbi@ti.com>,
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2  of
 * the License as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/platform_device.h>

#include "core.h"
#include "xhci.h"

int dwc3_host_init(struct dwc3 *dwc)
{
	struct platform_device	*xhci;
	int			ret;
	struct xhci_plat_data	pdata;

	xhci = platform_device_alloc("xhci-hcd", PLATFORM_DEVID_AUTO);
	if (!xhci) {
		dev_err(dwc->dev, "couldn't allocate xHCI device\n");
		ret = -ENOMEM;
		goto err0;
	}

	dma_set_coherent_mask(&xhci->dev, dwc->dev->coherent_dma_mask);

	xhci->dev.dma_mask	= dwc->dev->dma_mask;
	xhci->dev.dma_parms	= dwc->dev->dma_parms;

	dwc->xhci = xhci;
	pdata.vendor = ((dwc->revision & DWC3_GSNPSID_MASK) >>
			__ffs(DWC3_GSNPSID_MASK) & DWC3_GSNPSREV_MASK);
	pdata.revision = dwc->revision & DWC3_GSNPSREV_MASK;

	ret = platform_device_add_data(xhci, (const void *) &pdata,
			sizeof(struct xhci_plat_data));
	if (ret) {
		dev_err(dwc->dev, "couldn't add pdata to xHCI device\n");
		goto err1;
	}

	ret = platform_device_add_resources(xhci, dwc->xhci_resources,
						DWC3_XHCI_RESOURCES_NUM);
	if (ret) {
		dev_err(dwc->dev, "couldn't add resources to xHCI device\n");
		goto err1;
	}

	/* Add XHCI device if !OTG, otherwise OTG takes care of this */
	if (!dwc->dotg) {
		ret = platform_device_add(xhci);
		if (ret) {
			dev_err(dwc->dev, "failed to register xHCI device\n");
			goto err1;
		}
	}

	return 0;

err1:
	platform_device_put(xhci);

err0:
	return ret;
}

void dwc3_host_exit(struct dwc3 *dwc)
{
	if (!dwc->dotg)
		platform_device_unregister(dwc->xhci);
}
