/* Copyright (c) 2012-2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <asm/dma-iommu.h>
#include <linux/iommu.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/sort.h>
#include "msm_vidc_debug.h"
#include "msm_vidc_resources.h"
#include "msm_vidc_res_parse.h"
#include "venus_boot.h"

enum clock_properties {
	CLOCK_PROP_HAS_SCALING = 1 << 0,
};

static size_t get_u32_array_num_elements(struct platform_device *pdev,
					char *name)
{
	struct device_node *np = pdev->dev.of_node;
	int len;
	size_t num_elements = 0;
	if (!of_get_property(np, name, &len)) {
		dprintk(VIDC_ERR, "Failed to read %s from device tree\n",
			name);
		goto fail_read;
	}

	num_elements = len / sizeof(u32);
	if (num_elements <= 0) {
		dprintk(VIDC_ERR, "%s not specified in device tree\n",
			name);
		goto fail_read;
	}
	return num_elements;

fail_read:
	return 0;
}

static inline enum imem_type read_imem_type(struct platform_device *pdev)
{
	bool is_compatible(char *compat)
	{
		return !!of_find_compatible_node(NULL, NULL, compat);
	}

	return is_compatible("qcom,msm-ocmem") ? IMEM_OCMEM :
		is_compatible("qcom,msm-vmem") ? IMEM_VMEM :
						IMEM_NONE;

}

int read_hfi_type(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int rc = 0;
	const char *hfi_name = NULL;

	if (np) {
		rc = of_property_read_string(np, "qcom,hfi", &hfi_name);
		if (rc) {
			dprintk(VIDC_ERR,
				"Failed to read hfi from device tree\n");
			goto err_hfi_read;
		}
		if (!strcasecmp(hfi_name, "venus"))
			rc = VIDC_HFI_VENUS;
		else if (!strcasecmp(hfi_name, "q6"))
			rc = VIDC_HFI_Q6;
		else
			rc = -EINVAL;
	} else
		rc = VIDC_HFI_Q6;

err_hfi_read:
	return rc;
}

static inline void msm_vidc_free_freq_table(
		struct msm_vidc_platform_resources *res)
{
	res->load_freq_tbl = NULL;
}

static inline void msm_vidc_free_reg_table(
			struct msm_vidc_platform_resources *res)
{
	res->reg_set.reg_tbl = NULL;
}

static inline void msm_vidc_free_qdss_addr_table(
			struct msm_vidc_platform_resources *res)
{
	res->qdss_addr_set.addr_tbl = NULL;
}

static inline void msm_vidc_free_bus_vectors(
			struct msm_vidc_platform_resources *res)
{
	int i = 0;
	for (i = 0; i < res->bus_set.count; i++) {
		if (res->bus_set.bus_tbl[i].pdata)
			msm_bus_cl_clear_pdata(res->bus_set.bus_tbl[i].pdata);
	}
}

static inline void msm_vidc_free_buffer_usage_table(
			struct msm_vidc_platform_resources *res)
{
	res->buffer_usage_set.buffer_usage_tbl = NULL;
}

static inline void msm_vidc_free_regulator_table(
			struct msm_vidc_platform_resources *res)
{
	int c = 0;
	for (c = 0; c < res->regulator_set.count; ++c) {
		struct regulator_info *rinfo =
			&res->regulator_set.regulator_tbl[c];

		kfree(rinfo->name);
		rinfo->name = NULL;
	}

	/* The regulator table is one the few allocs that aren't managed, hence
	 * free it manually */
	kfree(res->regulator_set.regulator_tbl);
	res->regulator_set.regulator_tbl = NULL;
	res->regulator_set.count = 0;
}

static inline void msm_vidc_free_clock_table(
			struct msm_vidc_platform_resources *res)
{
	res->clock_set.clock_tbl = NULL;
	res->clock_set.count = 0;
}

void msm_vidc_free_platform_resources(
			struct msm_vidc_platform_resources *res)
{
	msm_vidc_free_clock_table(res);
	msm_vidc_free_regulator_table(res);
	msm_vidc_free_freq_table(res);
	msm_vidc_free_reg_table(res);
	msm_vidc_free_qdss_addr_table(res);
	msm_vidc_free_bus_vectors(res);
	msm_vidc_free_buffer_usage_table(res);
}

static int msm_vidc_load_reg_table(struct msm_vidc_platform_resources *res)
{
	struct reg_set *reg_set;
	struct platform_device *pdev = res->pdev;
	int i;
	int rc = 0;

	if (!of_find_property(pdev->dev.of_node, "qcom,reg-presets", NULL)) {
		/* qcom,reg-presets is an optional property.  It likely won't be
		 * present if we don't have any register settings to program */
		dprintk(VIDC_DBG, "qcom,reg-presets not found\n");
		return 0;
	}

	reg_set = &res->reg_set;
	reg_set->count = get_u32_array_num_elements(pdev, "qcom,reg-presets");
	reg_set->count /=  sizeof(*reg_set->reg_tbl) / sizeof(u32);

	if (!reg_set->count) {
		dprintk(VIDC_DBG, "no elements in reg set\n");
		return rc;
	}

	reg_set->reg_tbl = devm_kzalloc(&pdev->dev, reg_set->count *
			sizeof(*(reg_set->reg_tbl)), GFP_KERNEL);
	if (!reg_set->reg_tbl) {
		dprintk(VIDC_ERR, "%s Failed to alloc register table\n",
			__func__);
		return -ENOMEM;
	}

	if (of_property_read_u32_array(pdev->dev.of_node, "qcom,reg-presets",
		(u32 *)reg_set->reg_tbl, reg_set->count * 2)) {
		dprintk(VIDC_ERR, "Failed to read register table\n");
		msm_vidc_free_reg_table(res);
		return -EINVAL;
	}
	for (i = 0; i < reg_set->count; i++) {
		dprintk(VIDC_DBG,
			"reg = %x, value = %x\n",
			reg_set->reg_tbl[i].reg,
			reg_set->reg_tbl[i].value
		);
	}
	return rc;
}
static int msm_vidc_load_qdss_table(struct msm_vidc_platform_resources *res)
{
	struct addr_set *qdss_addr_set;
	struct platform_device *pdev = res->pdev;
	int i;
	int rc = 0;

	if (!of_find_property(pdev->dev.of_node, "qcom,qdss-presets", NULL)) {
		/* qcom,qdss-presets is an optional property. It likely won't be
		 * present if we don't have any register settings to program */
		dprintk(VIDC_DBG, "qcom,qdss-presets not found\n");
		return rc;
	}

	qdss_addr_set = &res->qdss_addr_set;
	qdss_addr_set->count = get_u32_array_num_elements(pdev,
					"qcom,qdss-presets");
	qdss_addr_set->count /= sizeof(*qdss_addr_set->addr_tbl) / sizeof(u32);

	if (!qdss_addr_set->count) {
		dprintk(VIDC_DBG, "no elements in qdss reg set\n");
		return rc;
	}

	qdss_addr_set->addr_tbl = devm_kzalloc(&pdev->dev,
			qdss_addr_set->count * sizeof(*qdss_addr_set->addr_tbl),
			GFP_KERNEL);
	if (!qdss_addr_set->addr_tbl) {
		dprintk(VIDC_ERR, "%s Failed to alloc register table\n",
			__func__);
		rc = -ENOMEM;
		goto err_qdss_addr_tbl;
	}

	rc = of_property_read_u32_array(pdev->dev.of_node, "qcom,qdss-presets",
		(u32 *)qdss_addr_set->addr_tbl, qdss_addr_set->count * 2);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to read qdss address table\n");
		msm_vidc_free_qdss_addr_table(res);
		rc = -EINVAL;
		goto err_qdss_addr_tbl;
	}

	for (i = 0; i < qdss_addr_set->count; i++) {
		dprintk(VIDC_DBG, "qdss addr = %x, value = %x\n",
				qdss_addr_set->addr_tbl[i].start,
				qdss_addr_set->addr_tbl[i].size);
	}
err_qdss_addr_tbl:
	return rc;
}

static int msm_vidc_load_freq_table(struct msm_vidc_platform_resources *res)
{
	int rc = 0;
	int num_elements = 0;
	struct platform_device *pdev = res->pdev;

	/* A comparator to compare loads (needed later on) */
	int cmp(const void *a, const void *b)
	{
		/* want to sort in reverse so flip the comparison */
		return ((struct load_freq_table *)b)->load -
			((struct load_freq_table *)a)->load;
	}

	if (!of_find_property(pdev->dev.of_node, "qcom,load-freq-tbl", NULL)) {
		/* qcom,load-freq-tbl is an optional property.  It likely won't
		 * be present on cores that we can't clock scale on. */
		dprintk(VIDC_DBG, "qcom,load-freq-tbl not found\n");
		return 0;
	}

	num_elements = get_u32_array_num_elements(pdev, "qcom,load-freq-tbl");
	num_elements /= sizeof(*res->load_freq_tbl) / sizeof(u32);
	if (!num_elements) {
		dprintk(VIDC_ERR, "no elements in frequency table\n");
		return rc;
	}

	res->load_freq_tbl = devm_kzalloc(&pdev->dev, num_elements *
			sizeof(*res->load_freq_tbl), GFP_KERNEL);
	if (!res->load_freq_tbl) {
		dprintk(VIDC_ERR,
				"%s Failed to alloc load_freq_tbl\n",
				__func__);
		return -ENOMEM;
	}

	if (of_property_read_u32_array(pdev->dev.of_node,
		"qcom,load-freq-tbl", (u32 *)res->load_freq_tbl,
		num_elements * sizeof(*res->load_freq_tbl) / sizeof(u32))) {
		dprintk(VIDC_ERR, "Failed to read frequency table\n");
		msm_vidc_free_freq_table(res);
		return -EINVAL;
	}

	res->load_freq_tbl_size = num_elements;

	/* The entries in the DT might not be sorted (for aesthetic purposes).
	 * Given that we expect the loads in descending order for our scaling
	 * logic to work, just sort it ourselves
	 */
	sort(res->load_freq_tbl, res->load_freq_tbl_size,
			sizeof(*res->load_freq_tbl), cmp, NULL);
	return rc;
}

static int msm_vidc_load_bus_vectors(struct msm_vidc_platform_resources *res)
{
	struct platform_device *pdev = res->pdev;
	struct device_node *child_node, *bus_node;
	struct bus_set *buses = &res->bus_set;
	int rc = 0, c = 0;
	u32 num_buses = 0;

	bus_node = of_find_node_by_name(pdev->dev.of_node,
			"qcom,msm-bus-clients");
	if (!bus_node) {
		/* Not a required property */
		dprintk(VIDC_DBG, "qcom,msm-bus-clients not found\n");
		rc = 0;
		goto err_bad_node;
	}

	for_each_child_of_node(bus_node, child_node)
		++num_buses;

	buses->bus_tbl = devm_kzalloc(&pdev->dev, sizeof(*buses->bus_tbl) *
			num_buses, GFP_KERNEL);
	if (!buses->bus_tbl) {
		dprintk(VIDC_ERR, "%s: Failed to allocate memory\n", __func__);
		rc = -ENOMEM;
		goto err_bad_node;
	}

	buses->count = num_buses;
	c = 0;

	for_each_child_of_node(bus_node, child_node) {
		bool passive = false;
		u32 configs = 0;
		struct bus_info *bus = &buses->bus_tbl[c];

		passive = of_property_read_bool(child_node, "qcom,bus-passive");
		rc = of_property_read_u32(child_node, "qcom,bus-configs",
				&configs);
		if (rc) {
			dprintk(VIDC_ERR,
					"Failed to read qcom,bus-configs in %s: %d\n",
					child_node->name, rc);
			break;
		}

		bus->passive = passive;
		bus->sessions_supported = configs;
		bus->pdata = msm_bus_pdata_from_node(pdev, child_node);
		if (IS_ERR_OR_NULL(bus->pdata)) {
			rc = PTR_ERR(bus->pdata) ?: -EBADHANDLE;
			dprintk(VIDC_ERR, "Failed to get bus pdata: %d\n", rc);
			break;
		}

		dprintk(VIDC_DBG, "Bus %s supports: %x, passive: %d\n",
				bus->pdata->name, bus->sessions_supported,
				passive);
		++c;
	}

	if (c < num_buses) {
		for (c--; c >= 0; c--)
			msm_bus_cl_clear_pdata(buses->bus_tbl[c].pdata);

		goto err_bad_node;
	}

err_bad_node:
	return rc;
}

static int msm_vidc_load_buffer_usage_table(
		struct msm_vidc_platform_resources *res)
{
	int rc = 0;
	struct platform_device *pdev = res->pdev;
	struct buffer_usage_set *buffer_usage_set = &res->buffer_usage_set;

	if (!of_find_property(pdev->dev.of_node,
				"qcom,buffer-type-tz-usage-table", NULL)) {
		/* qcom,buffer-type-tz-usage-table is an optional property.  It
		 * likely won't be present if the core doesn't support content
		 * protection */
		dprintk(VIDC_DBG, "buffer-type-tz-usage-table not found\n");
		return 0;
	}

	buffer_usage_set->count = get_u32_array_num_elements(
				    pdev, "qcom,buffer-type-tz-usage-table");
	buffer_usage_set->count /=
		sizeof(*buffer_usage_set->buffer_usage_tbl) / sizeof(u32);
	if (!buffer_usage_set->count) {
		dprintk(VIDC_DBG, "no elements in buffer usage set\n");
		return 0;
	}

	buffer_usage_set->buffer_usage_tbl = devm_kzalloc(&pdev->dev,
			buffer_usage_set->count *
			sizeof(*buffer_usage_set->buffer_usage_tbl),
			GFP_KERNEL);
	if (!buffer_usage_set->buffer_usage_tbl) {
		dprintk(VIDC_ERR, "%s Failed to alloc buffer usage table\n",
			__func__);
		rc = -ENOMEM;
		goto err_load_buf_usage;
	}

	rc = of_property_read_u32_array(pdev->dev.of_node,
		    "qcom,buffer-type-tz-usage-table",
		(u32 *)buffer_usage_set->buffer_usage_tbl,
		buffer_usage_set->count *
		sizeof(*buffer_usage_set->buffer_usage_tbl) / sizeof(u32));
	if (rc) {
		dprintk(VIDC_ERR, "Failed to read buffer usage table\n");
		goto err_load_buf_usage;
	}

	return 0;
err_load_buf_usage:
	msm_vidc_free_buffer_usage_table(res);
	return rc;
}

static int msm_vidc_load_regulator_table(
		struct msm_vidc_platform_resources *res)
{
	int rc = 0;
	struct platform_device *pdev = res->pdev;
	struct regulator_set *regulators = &res->regulator_set;
	struct device_node *domains_parent_node = NULL;
	struct property *domains_property = NULL;

	regulators->count = 0;
	regulators->regulator_tbl = NULL;

	domains_parent_node = pdev->dev.of_node;
	for_each_property_of_node(domains_parent_node, domains_property) {
		const char *search_string = "-supply";
		char *supply;
		bool matched = false;
		struct device_node *regulator_node = NULL;
		struct regulator_info *rinfo = NULL;
		void *temp = NULL;

		/* 1) check if current property is possibly a regulator */
		supply = strnstr(domains_property->name, search_string,
				strlen(domains_property->name) + 1);
		matched = supply && (*(supply + strlen(search_string)) == '\0');
		if (!matched)
			continue;

		/* 2) make sure prop isn't being misused */
		regulator_node = of_parse_phandle(domains_parent_node,
				domains_property->name, 0);
		if (IS_ERR(regulator_node)) {
			dprintk(VIDC_WARN, "%s is not a phandle\n",
					domains_property->name);
			continue;
		}

		/* 3) expand our table */
		temp = krealloc(regulators->regulator_tbl,
				sizeof(*regulators->regulator_tbl) *
				(regulators->count + 1), GFP_KERNEL);
		if (!temp) {
			rc = -ENOMEM;
			dprintk(VIDC_ERR,
					"Failed to alloc memory for regulator table\n");
			goto err_reg_tbl_alloc;
		}

		regulators->regulator_tbl = temp;
		regulators->count++;

		/* 4) populate regulator info */
		rinfo = &regulators->regulator_tbl[regulators->count - 1];
		rinfo->name = kstrndup(domains_property->name,
				supply - domains_property->name, GFP_KERNEL);
		if (!rinfo->name) {
			rc = -ENOMEM;
			dprintk(VIDC_ERR,
					"Failed to alloc memory for regulator name\n");
			goto err_reg_name_alloc;
		}

		rinfo->has_hw_power_collapse = of_property_read_bool(
			regulator_node, "qcom,support-hw-trigger");

		dprintk(VIDC_DBG, "Found regulator %s: h/w collapse = %s\n",
				rinfo->name,
				rinfo->has_hw_power_collapse ? "yes" : "no");
	}

	if (!regulators->count)
		dprintk(VIDC_DBG, "No regulators found");

	return 0;

err_reg_name_alloc:
err_reg_tbl_alloc:
	msm_vidc_free_regulator_table(res);
	return rc;
}

static int msm_vidc_load_clock_table(
		struct msm_vidc_platform_resources *res)
{
	int rc = 0, num_clocks = 0, c = 0;
	struct platform_device *pdev = res->pdev;
	int *clock_props = NULL;
	struct clock_set *clocks = &res->clock_set;

	num_clocks = of_property_count_strings(pdev->dev.of_node,
				"clock-names");
	if (num_clocks <= 0) {
		/* Devices such as Q6 might not have any control over clocks
		 * hence have none specified, which is ok. */
		dprintk(VIDC_DBG, "No clocks found\n");
		clocks->count = 0;
		rc = 0;
		goto err_load_clk_table_fail;
	}

	clock_props = devm_kzalloc(&pdev->dev, num_clocks *
			sizeof(*clock_props), GFP_KERNEL);
	if (!clock_props) {
		dprintk(VIDC_ERR, "No memory to read clock properties\n");
		rc = -ENOMEM;
		goto err_load_clk_table_fail;
	}

	rc = of_property_read_u32_array(pdev->dev.of_node,
				"qcom,clock-configs", clock_props,
				num_clocks);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to read clock properties: %d\n", rc);
		goto err_load_clk_prop_fail;
	}

	clocks->clock_tbl = devm_kzalloc(&pdev->dev, sizeof(*clocks->clock_tbl)
			* num_clocks, GFP_KERNEL);
	if (!clocks->clock_tbl) {
		dprintk(VIDC_ERR, "Failed to allocate memory for clock tbl\n");
		rc = -ENOMEM;
		goto err_load_clk_prop_fail;
	}

	clocks->count = num_clocks;
	dprintk(VIDC_DBG, "Found %d clocks\n", num_clocks);

	for (c = 0; c < num_clocks; ++c) {
		struct clock_info *vc = &res->clock_set.clock_tbl[c];

		of_property_read_string_index(pdev->dev.of_node,
				"clock-names", c, &vc->name);

		if (clock_props[c] & CLOCK_PROP_HAS_SCALING) {
			vc->count = res->load_freq_tbl_size;
			vc->load_freq_tbl = res->load_freq_tbl;
		} else {
			vc->count = 0;
			vc->load_freq_tbl = NULL;
		}

		dprintk(VIDC_DBG, "Found clock %s: scale-able = %s\n", vc->name,
			vc->count ? "yes" : "no");
	}


	return 0;

err_load_clk_prop_fail:
err_load_clk_table_fail:
	return rc;
}

int read_platform_resources_from_dt(
		struct msm_vidc_platform_resources *res)
{
	struct platform_device *pdev = res->pdev;
	struct resource *kres = NULL;
	int rc = 0;
	uint32_t firmware_base = 0;

	if (!pdev->dev.of_node) {
		dprintk(VIDC_ERR, "DT node not found\n");
		return -ENOENT;
	}

	INIT_LIST_HEAD(&res->context_banks);

	res->firmware_base = (phys_addr_t)firmware_base;

	kres = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	res->register_base = kres ? kres->start : -1;
	res->register_size = kres ? (kres->end + 1 - kres->start) : -1;

	kres = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	res->irq = kres ? kres->start : -1;

	of_property_read_u32(pdev->dev.of_node,
			"qcom,imem-size", &res->imem_size);
	res->imem_type = read_imem_type(pdev);

	res->sys_idle_indicator = of_property_read_bool(pdev->dev.of_node,
			"qcom,enable-idle-indicator");

	res->thermal_mitigable =
			of_property_read_bool(pdev->dev.of_node,
			"qcom,enable-thermal-mitigation");

	rc = of_property_read_string(pdev->dev.of_node, "qcom,firmware-name",
			&res->fw_name);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to read firmware name: %d\n", rc);
		goto err_load_freq_table;
	}
	dprintk(VIDC_DBG, "Firmware filename: %s\n", res->fw_name);

	rc = of_property_read_string(pdev->dev.of_node, "qcom,hfi-version",
			&res->hfi_version);
	if (rc)
		dprintk(VIDC_DBG, "HFI packetization will default to legacy\n");

	rc = msm_vidc_load_freq_table(res);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to load freq table: %d\n", rc);
		goto err_load_freq_table;
	}

	rc = msm_vidc_load_qdss_table(res);
	if (rc)
		dprintk(VIDC_WARN, "Failed to load qdss reg table: %d\n", rc);

	rc = msm_vidc_load_reg_table(res);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to load reg table: %d\n", rc);
		goto err_load_reg_table;
	}

	rc = msm_vidc_load_bus_vectors(res);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to load bus vectors: %d\n", rc);
		goto err_load_bus_vectors;
	}
	rc = msm_vidc_load_buffer_usage_table(res);
	if (rc) {
		dprintk(VIDC_ERR,
			"Failed to load buffer usage table: %d\n", rc);
		goto err_load_buffer_usage_table;
	}

	rc = msm_vidc_load_regulator_table(res);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to load list of regulators %d\n", rc);
		goto err_load_regulator_table;
	}

	rc = msm_vidc_load_clock_table(res);
	if (rc) {
		dprintk(VIDC_ERR,
			"Failed to load clock table: %d\n", rc);
		goto err_load_clock_table;
	}

	rc = of_property_read_u32(pdev->dev.of_node, "qcom,max-hw-load",
			&res->max_load);
	if (rc) {
		dprintk(VIDC_ERR,
			"Failed to determine max load supported: %d\n", rc);
		goto err_load_max_hw_load;
	}

	res->use_non_secure_pil = of_property_read_bool(pdev->dev.of_node,
			"qcom,use-non-secure-pil");

	if (res->use_non_secure_pil || !is_iommu_present(res)) {
		of_property_read_u32(pdev->dev.of_node, "qcom,fw-bias",
				&firmware_base);
		res->firmware_base = (phys_addr_t)firmware_base;
		dprintk(VIDC_DBG,
				"Using fw-bias : %pa", &res->firmware_base);
	}

	res->sw_power_collapsible = of_property_read_bool(pdev->dev.of_node,
					"qcom,sw-power-collapse");
	dprintk(VIDC_DBG, "Power collapse supported = %s\n",
		res->sw_power_collapsible ? "yes" : "no");

	res->early_fw_load = of_property_read_bool(pdev->dev.of_node,
				"qcom,early-fw-load");
	dprintk(VIDC_DBG, "Early fw load = %s\n",
				res->early_fw_load ? "yes" : "no");
	return rc;
err_load_max_hw_load:
	msm_vidc_free_clock_table(res);
err_load_clock_table:
	msm_vidc_free_regulator_table(res);
err_load_regulator_table:
	msm_vidc_free_buffer_usage_table(res);
err_load_buffer_usage_table:
	msm_vidc_free_bus_vectors(res);
err_load_bus_vectors:
	msm_vidc_free_reg_table(res);
err_load_reg_table:
	msm_vidc_free_freq_table(res);
err_load_freq_table:
	return rc;
}

static int msm_vidc_setup_context_bank(struct context_bank_info *cb,
		struct device *dev)
{
	int rc = 0;
	int order = 0;
	bool disable_htw = true;

	if (!dev || !cb) {
		dprintk(VIDC_ERR,
			"%s: Invalid Input params\n", __func__);
		return -EINVAL;
	}

	cb->dev = dev;
	cb->mapping = arm_iommu_create_mapping(&platform_bus_type,
			cb->addr_range.start, cb->addr_range.size, order);

	if (IS_ERR_OR_NULL(cb->mapping)) {
		dprintk(VIDC_ERR, "%s - failed to create mapping\n", __func__);
		rc = PTR_ERR(cb->mapping) ?: -ENODEV;
		goto remove_cb;
	}

	rc = arm_iommu_attach_device(cb->dev, cb->mapping);
	if (rc) {
		dprintk(VIDC_ERR, "%s - Couldn't arm_iommu_attach_device\n",
			__func__);
		goto release_mapping;
	}

	rc = iommu_domain_set_attr(cb->mapping->domain,
			DOMAIN_ATTR_COHERENT_HTW_DISABLE, &disable_htw);
	if (rc) {
		dprintk(VIDC_ERR, "%s - disable coherent HTW failed: %s %d\n",
				__func__, dev_name(dev), rc);
		goto detach_device;
	}

	dprintk(VIDC_DBG, "Attached %s and created mapping\n", dev_name(dev));
	dprintk(VIDC_DBG,
		"Context bank name:%s, buffer_type: %#x, is_secure: %d, address range start: %#x, size: %#x, dev: %p, mapping: %p",
		cb->name, cb->buffer_type, cb->is_secure, cb->addr_range.start,
		cb->addr_range.size, cb->dev, cb->mapping);

	return rc;

detach_device:
	arm_iommu_detach_device(cb->dev);
release_mapping:
	arm_iommu_release_mapping(cb->mapping);
remove_cb:
	return rc;
}

static int msm_vidc_populate_context_bank(struct device *dev,
		struct msm_vidc_platform_resources *res)
{
	int rc = 0;
	struct context_bank_info *cb = NULL;
	struct device_node *np = NULL;

	if (!dev || !res) {
		dprintk(VIDC_ERR, "%s - invalid inputs\n", __func__);
		return -EINVAL;
	}

	np = dev->of_node;
	cb = devm_kzalloc(dev, sizeof(*cb), GFP_KERNEL);
	if (!cb) {
		dprintk(VIDC_ERR, "%s - Failed to allocate cb\n", __func__);
		return -ENOMEM;
	}

	INIT_LIST_HEAD(&cb->list);
	list_add_tail(&cb->list, &res->context_banks);

	rc = of_property_read_string(np, "label", &cb->name);
	if (rc) {
		dprintk(VIDC_DBG,
			"Failed to read cb label from device tree\n");
		rc = 0;
	}

	dprintk(VIDC_DBG, "%s: context bank has name %s\n", __func__, cb->name);
	rc = of_property_read_u32_array(np, "virtual-addr-pool",
			(u32 *)&cb->addr_range, 2);
	if (rc) {
		dprintk(VIDC_ERR,
			"Could not read addr pool for context bank : %s %d\n",
			cb->name, rc);
		goto err_setup_cb;
	}

	cb->is_secure = of_property_read_bool(np, "secure-addr-range");
	dprintk(VIDC_DBG, "context bank %s : secure = %d\n",
			cb->name, cb->is_secure);

	/* setup buffer type for each sub device*/
	rc = of_property_read_u32(np, "buffer-types", &cb->buffer_type);
	if (rc) {
		dprintk(VIDC_ERR, "failed to load buffer_type info %d\n", rc);
		rc = -ENOENT;
		goto err_setup_cb;
	}
	dprintk(VIDC_DBG,
		"context bank %s address start = %x address size = %x buffer_type = %x\n",
		cb->name, cb->addr_range.start,
		cb->addr_range.size, cb->buffer_type);

	rc = msm_vidc_setup_context_bank(cb, dev);
	if (rc) {
		dprintk(VIDC_ERR, "Cannot setup context bank %d\n", rc);
		goto err_setup_cb;
	}

	return 0;

err_setup_cb:
	list_del(&cb->list);
	return rc;
}

int msm_vidc_probe_sub_devices(struct platform_device *pdev)
{
	struct msm_vidc_core *core;
	int rc = 0;

	if (!pdev) {
		dprintk(VIDC_ERR, "Invalid platform device\n");
		return -EINVAL;
	} else if (!pdev->dev.parent) {
		dprintk(VIDC_ERR, "Failed to find a parent for %s\n",
				dev_name(&pdev->dev));
		return -ENODEV;
	}

	core = dev_get_drvdata(pdev->dev.parent);
	if (!core) {
		dprintk(VIDC_ERR, "Failed to find cookie in parent device %s",
				dev_name(pdev->dev.parent));
		return -EINVAL;
	}

	dprintk(VIDC_DBG, "Probing %s\n", dev_name(&pdev->dev));
	if (of_property_read_bool(pdev->dev.of_node, "qcom,fw-context-bank")) {
		if (core->resources.use_non_secure_pil) {
			struct context_bank_info *cb;

			cb = devm_kzalloc(&pdev->dev, sizeof(*cb), GFP_KERNEL);
			if (!cb) {
				dprintk(VIDC_ERR, "alloc venus cb failed\n");
				return -ENOMEM;
			}

			cb->dev = &pdev->dev;
			rc = venus_boot_init(&core->resources, cb);
			if (rc) {
				dprintk(VIDC_ERR,
				"Failed to init non-secure PIL %d\n", rc);
			}
		}
	} else {
		rc = msm_vidc_populate_context_bank(&pdev->dev,
					&core->resources);
		if (rc)
			dprintk(VIDC_ERR, "Failed to probe context bank\n");
		else
			dprintk(VIDC_DBG, "Successfully probed context bank\n");
	}
	return rc;
}
