/*
 * drivers/video/tegra/host/gk20a/gk20a.c
 *
 * GK20A Graphics
 *
 * Copyright (c) 2011-2012, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/highmem.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/export.h>
#include <asm/cacheflush.h>

#include "dev.h"
#include "class_ids.h"
#include "bus_client.h"
#include "nvhost_as.h"

#include "gk20a.h"
#include "ctrl_gk20a.h"
#include "hw_mc_gk20a.h"
#include "hw_sim_gk20a.h"

#include "../../../../../../arch/arm/mach-tegra/iomap.h"

static inline void set_gk20a(struct platform_device *dev, struct gk20a *gk20a)
{
	nvhost_set_private_data(dev, gk20a);
}


#define APBDEV_PMC_GPU_RG_CNTRL_0	0x448

static void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);

static void nvhost_gk20a_deinit(struct platform_device *dev);
static struct nvhost_hwctx_handler *
    nvhost_gk20a_alloc_hwctx_handler(u32 syncpt, u32 base,
				     struct nvhost_channel *ch);

/* TBD: should be able to put in the list below. */
static struct resource gk20a_intr = {
	.start = TEGRA_GK20A_INTR,
	.end   = TEGRA_GK20A_INTR_NONSTALL,
	.flags = IORESOURCE_IRQ,
};

struct resource gk20a_resources [] = {
#define GK20A_BAR0_IORESOURCE_MEM 0
{
	.start = TEGRA_GK20A_BAR0_BASE,
	.end   = TEGRA_GK20A_BAR0_BASE + TEGRA_GK20A_BAR0_SIZE - 1,
	.flags = IORESOURCE_MEM,
},
#define GK20A_BAR1_IORESOURCE_MEM 1
{
	.start = TEGRA_GK20A_BAR1_BASE,
	.end   = TEGRA_GK20A_BAR1_BASE + TEGRA_GK20A_BAR1_SIZE - 1,
	.flags = IORESOURCE_MEM,
},
#if CONFIG_GK20A_SIM
#define GK20A_SIM_IORESOURCE_MEM 2
{
#define TEGRA_GK20A_SIM_BASE 0x538F0000 /*tbd: get from iomap.h should get this or replacement */
#define TEGRA_GK20A_SIM_SIZE 0x1000     /*tbd: this is a high-side guess */
	.start = TEGRA_GK20A_SIM_BASE,
	.end   = TEGRA_GK20A_SIM_BASE + TEGRA_GK20A_SIM_SIZE - 1,
	.flags = IORESOURCE_MEM,
},
#endif
};

static const struct file_operations gk20a_ctrl_ops = {
	.owner = THIS_MODULE,
	.release = gk20a_ctrl_dev_release,
	.open = gk20a_ctrl_dev_open,
	.unlocked_ioctl = gk20a_ctrl_dev_ioctl,
};

static struct nvhost_device_data tegra_gk20a_info = {
	/* the following are set by the platform (e.g. t124) support
	.syncpts       = BIT(NVSYNCPT_3D),
	.waitbases     = BIT(NVWAITBASE_3D),
	.modulemutexes = BIT(NVMODMUTEX_3D),
	*/
	.class	       = NV_GRAPHICS_GPU_CLASS_ID,
	.keepalive     = true,
	.clocks = {{"emc", UINT_MAX}, {}},
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.alloc_hwctx_handler = nvhost_gk20a_alloc_hwctx_handler,
	.ctrl_ops = &gk20a_ctrl_ops,
	.moduleid      = NVHOST_MODULE_GPU,
};

struct platform_device tegra_gk20a_device = {
	.name          = "gk20a",
	.resource      = gk20a_resources,
#if CONFIG_GK20A_SIM
	.num_resources = 3, /* this is num ioresource_mem, not the sum */
#else
	.num_resources = 2, /* this is num ioresource_mem, not the sum */
#endif
	.dev           = {
		.platform_data = &tegra_gk20a_info,
	},
};



#if CONFIG_GK20A_SIM
static inline void sim_writel(struct gk20a *g, u32 r, u32 v)
{
	writel(v, g->sim.regs+r);
}
static inline u32 sim_readl(struct gk20a *g, u32 r)
{
	return readl(g->sim.regs+r);
}

static void kunmap_and_free_iopage(void **kvaddr, struct page **page)
{
	if (*kvaddr) {
		kunmap(*kvaddr);
		*kvaddr = 0;
	}
	if (*page) {
		__free_page(*page);
		*page = 0;
	}
}

static void gk20a_free_sim_support(struct gk20a *g)
{
	/* free sim mappings, bfrs */
	kunmap_and_free_iopage(&g->sim.send_bfr.kvaddr,
			       &g->sim.send_bfr.page);

	kunmap_and_free_iopage(&g->sim.recv_bfr.kvaddr,
			       &g->sim.recv_bfr.page);

	kunmap_and_free_iopage(&g->sim.msg_bfr.kvaddr,
			       &g->sim.msg_bfr.page);
}

static void gk20a_remove_sim_support(struct sim_gk20a *s)
{
	struct gk20a *g = s->g;
	if (g->sim.regs)
		sim_writel(g, sim_config_r(), sim_config_mode_disabled_v());
	gk20a_free_sim_support(g);
}

static int alloc_and_kmap_iopage(struct device *d,
				 void **kvaddr,
				 phys_addr_t *phys,
				 struct page **page)
{
	int err = 0;
	*page = alloc_page(GFP_KERNEL);

	if (!*page) {
		err = -ENOMEM;
		dev_err(d, "couldn't allocate io page\n");
		goto fail;
	}

	*kvaddr = kmap(*page);
	if (!*kvaddr) {
		err = -ENOMEM;
		dev_err(d, "couldn't kmap io page\n");
		goto fail;
	}
	*phys = page_to_phys(*page);
	return 0;

 fail:
	kunmap_and_free_iopage(kvaddr, page);
	return err;

}
/* TBD: strip from released */
static int gk20a_init_sim_support(struct platform_device *dev)
{
	int err = 0;
	struct gk20a *g = get_gk20a(dev);
	struct nvhost_device_data *pdata = nvhost_get_devdata(dev);
	struct device *d = &dev->dev;
	phys_addr_t phys;

	g->sim.g = g;
	g->sim.regs = pdata->aperture[GK20A_SIM_IORESOURCE_MEM];
	if (!g->sim.regs) {
		dev_err(d, "failed to remap gk20a sim regs\n");
		err = -ENXIO;
		goto fail;
	}

	/* allocate sim event/msg buffers */
	err = alloc_and_kmap_iopage(d, &g->sim.send_bfr.kvaddr,
				    &g->sim.send_bfr.phys,
				    &g->sim.send_bfr.page);

	err = err || alloc_and_kmap_iopage(d, &g->sim.recv_bfr.kvaddr,
					   &g->sim.recv_bfr.phys,
					   &g->sim.recv_bfr.page);

	err = err || alloc_and_kmap_iopage(d, &g->sim.msg_bfr.kvaddr,
					   &g->sim.msg_bfr.phys,
					   &g->sim.msg_bfr.page);

	if (!(g->sim.send_bfr.kvaddr && g->sim.recv_bfr.kvaddr &&
	      g->sim.msg_bfr.kvaddr)) {
		dev_err(d, "couldn't allocate all sim buffers\n");
		goto fail;
	}

	/*mark send ring invalid*/
	sim_writel(g, sim_send_ring_r(), sim_send_ring_status_invalid_f());

	/*read get pointer and make equal to put*/
	g->sim.send_ring_put = sim_readl(g, sim_send_get_r());
	sim_writel(g, sim_send_put_r(), g->sim.send_ring_put);

	/*write send ring address and make it valid*/
	/*TBD: work for >32b physmem*/
	BUILD_BUG_ON(sizeof(phys_addr_t) != sizeof(u32));
	phys = g->sim.send_bfr.phys;
	sim_writel(g, sim_send_ring_hi_r(), 0);
	sim_writel(g, sim_send_ring_r(),
		   sim_send_ring_status_valid_f() |
		   sim_send_ring_target_phys_pci_coherent_f() |
		   sim_send_ring_size_4kb_f() |
		   sim_send_ring_addr_lo_f(phys >> PAGE_SHIFT));

	/*repeat for recv ring (but swap put,get as roles are opposite) */
	sim_writel(g, sim_recv_ring_r(), sim_recv_ring_status_invalid_f());

	/*read put pointer and make equal to get*/
	g->sim.recv_ring_get = sim_readl(g, sim_recv_put_r());
	sim_writel(g, sim_recv_get_r(), g->sim.recv_ring_get);

	/*write send ring address and make it valid*/
	/*TBD: work for >32b physmem*/
	BUILD_BUG_ON(sizeof(phys_addr_t) != sizeof(u32));
	phys = g->sim.recv_bfr.phys;
	sim_writel(g, sim_recv_ring_hi_r(), 0);
	sim_writel(g, sim_recv_ring_r(),
		   sim_recv_ring_status_valid_f() |
		   sim_recv_ring_target_phys_pci_coherent_f() |
		   sim_recv_ring_size_4kb_f() |
		   sim_recv_ring_addr_lo_f(phys >> PAGE_SHIFT));

	g->sim.remove_support = gk20a_remove_sim_support;
	return 0;

 fail:
	gk20a_free_sim_support(g);
	return err;
}

static inline u32 sim_msg_header_size(void)
{
	return 24;/*TBD: fix the header to gt this from NV_VGPU_MSG_HEADER*/
}

static inline u32 *sim_msg_bfr(struct gk20a *g, u32 byte_offset)
{
	return (u32 *)(g->sim.msg_bfr.kvaddr + byte_offset);
}

static inline u32 *sim_msg_hdr(struct gk20a *g, u32 byte_offset)
{
	return sim_msg_bfr(g, byte_offset); /*starts at 0*/
}

static inline u32 *sim_msg_param(struct gk20a *g, u32 byte_offset)
{
	/*starts after msg header/cmn*/
	return sim_msg_bfr(g, byte_offset + sim_msg_header_size());
}

static inline void sim_write_hdr(struct gk20a *g, u32 func, u32 size)
{
	/*memset(g->sim.msg_bfr.kvaddr,0,min(PAGE_SIZE,size));*/
	*sim_msg_hdr(g, sim_msg_signature_r()) = sim_msg_signature_valid_v();
	*sim_msg_hdr(g, sim_msg_result_r())    = sim_msg_result_rpc_pending_v();
	*sim_msg_hdr(g, sim_msg_spare_r())     = sim_msg_spare__init_v();
	*sim_msg_hdr(g, sim_msg_function_r())  = func;
	*sim_msg_hdr(g, sim_msg_length_r())    = size + sim_msg_header_size();
}

static inline u32 sim_escape_read_hdr_size(void)
{
	return 12; /*TBD: fix NV_VGPU_SIM_ESCAPE_READ_HEADER*/
}
static u32 *sim_send_ring_bfr(struct gk20a *g, u32 byte_offset)
{
	return (u32 *)(g->sim.send_bfr.kvaddr + byte_offset);
}
static int rpc_send_message(struct gk20a *g)
{
	/* calculations done in units of u32s */
	u32 send_base = sim_send_put_pointer_v(g->sim.send_ring_put) * 2;
	u32 dma_offset = send_base + sim_dma_r()/sizeof(u32);
	u32 dma_hi_offset = send_base + sim_dma_hi_r()/sizeof(u32);

	*sim_send_ring_bfr(g, dma_offset*sizeof(u32)) =
		sim_dma_target_phys_pci_coherent_f() |
		sim_dma_status_valid_f() |
		sim_dma_size_4kb_f() |
		sim_dma_addr_lo_f(g->sim.msg_bfr.phys >> PAGE_SHIFT);

	*sim_send_ring_bfr(g, dma_hi_offset*sizeof(u32)) = 0; /*TBD >32b phys*/

	*sim_msg_hdr(g, sim_msg_sequence_r()) = g->sim.sequence_base++;

	g->sim.send_ring_put = (g->sim.send_ring_put + 2 * sizeof(u32)) %
		PAGE_SIZE;

	__cpuc_flush_dcache_area(g->sim.msg_bfr.kvaddr, PAGE_SIZE);
	__cpuc_flush_dcache_area(g->sim.send_bfr.kvaddr, PAGE_SIZE);
	__cpuc_flush_dcache_area(g->sim.recv_bfr.kvaddr, PAGE_SIZE);

	/* Update the put pointer. This will trap into the host. */
	sim_writel(g, sim_send_put_r(), g->sim.send_ring_put);

	return 0;
}

static inline u32 *sim_recv_ring_bfr(struct gk20a *g, u32 byte_offset)
{
	return (u32 *)(g->sim.recv_bfr.kvaddr + byte_offset);
}

static int rpc_recv_poll(struct gk20a *g)
{
	phys_addr_t recv_phys_addr;

	/* XXX This read is not required (?) */
	/*pVGpu->recv_ring_get = VGPU_REG_RD32(pGpu, NV_VGPU_RECV_GET);*/

	/* Poll the recv ring get pointer in an infinite loop*/
	do {
		g->sim.recv_ring_put = sim_readl(g, sim_recv_put_r());
	} while (g->sim.recv_ring_put == g->sim.recv_ring_get);

	/* process all replies */
	while (g->sim.recv_ring_put != g->sim.recv_ring_get) {
		/* these are in u32 offsets*/
		u32 dma_lo_offset =
			sim_recv_put_pointer_v(g->sim.recv_ring_get)*2 + 0;
		/*u32 dma_hi_offset = dma_lo_offset + 1;*/
		u32 recv_phys_addr_lo =	sim_dma_addr_lo_v(*sim_recv_ring_bfr(g, dma_lo_offset*4));

		/*u32 recv_phys_addr_hi = sim_dma_hi_addr_v(
		      (phys_addr_t)sim_recv_ring_bfr(g,dma_hi_offset*4));*/

		/*TBD >32b phys addr */
		recv_phys_addr = recv_phys_addr_lo << PAGE_SHIFT;

		if (recv_phys_addr != g->sim.msg_bfr.phys) {
			dev_err(dev_from_gk20a(g), "%s Error in RPC reply\n",
				__func__);
			return -1;
		}

		/* Update GET pointer */
		g->sim.recv_ring_get = (g->sim.recv_ring_get + 2*sizeof(u32)) %
			PAGE_SIZE;

		__cpuc_flush_dcache_area(g->sim.msg_bfr.kvaddr, PAGE_SIZE);
		__cpuc_flush_dcache_area(g->sim.send_bfr.kvaddr, PAGE_SIZE);
		__cpuc_flush_dcache_area(g->sim.recv_bfr.kvaddr, PAGE_SIZE);

		sim_writel(g, sim_recv_get_r(), g->sim.recv_ring_get);

		g->sim.recv_ring_put = sim_readl(g, sim_recv_put_r());
	}

	return 0;
}


static int issue_rpc_and_wait(struct gk20a *g)
{
	int err;

	err = rpc_send_message(g);
	if (err) {
		dev_err(dev_from_gk20a(g), "%s failed rpc_send_message\n",
			__func__);
		return err;
	}

	err = rpc_recv_poll(g);
	if (err) {
		dev_err(dev_from_gk20a(g), "%s failed rpc_recv_poll\n",
			__func__);
		return err;
	}

	/* Now check if RPC really succeeded */
	if (*sim_msg_hdr(g, sim_msg_result_r()) != sim_msg_result_success_v()) {
		dev_err(dev_from_gk20a(g), "%s received failed status!\n",
			__func__);
		return -(*sim_msg_hdr(g, sim_msg_result_r()));
	}
	return 0;
}

int gk20a_sim_esc_read(struct gk20a *g, char*path, u32 index, u32 count, u32 *data)
{
	int err;
	size_t pathlen = strlen(path);
	u32 data_offset;

	sim_write_hdr(g, sim_msg_function_sim_escape_read_v(),
		      sim_escape_read_hdr_size());
	*sim_msg_param(g, 0) = index;
	*sim_msg_param(g, 4) = count;
	data_offset = roundup(0xc +  pathlen + 1, sizeof(u32));
	*sim_msg_param(g, 8) = data_offset;
	strcpy((char *)sim_msg_param(g, 0xc), path);

	err = issue_rpc_and_wait(g);

	if (!err)
		memcpy(data, sim_msg_param(g, data_offset), count);
	return err;
}


#else /*CONFIG_GK20A_SIM*/
static inline int gk20a_init_sim_support(struct platform_device *dev);
{
	return 0;
}
static void gk20a_remove_sim_support(struct platform_device *dev)
{
}
#endif /*!CONFIG_GK20A_SIM*/

static irqreturn_t gk20a_intr_isr(int irq, void *dev_id)
{
	struct gk20a *g = dev_id;
	u32 mc_intr_0 = gk20a_readl(g, mc_intr_0_r());

	/* not from gpu when sharing irq with others */
	if (unlikely(!mc_intr_0))
		return IRQ_NONE;

	gk20a_writel(g, mc_intr_en_0_r(),
		mc_intr_en_0_inta_disabled_f());

	/* flush previous write */
	gk20a_readl(g, mc_intr_en_0_r());

	return IRQ_WAKE_THREAD;
}

static irqreturn_t gk20a_intr_thread(int irq, void *dev_id)
{
	struct gk20a *g = dev_id;
	u32 mc_intr_0;
	u32 loop = 0;

	nvhost_dbg(dbg_intr, "interrupt thread launched");

	mc_intr_0 = gk20a_readl(g, mc_intr_0_r());

	/* loop as many as three more times in case new interrupts come up
	 * while we're processing current ones */
	while (mc_intr_0 && loop++ < 3) {
		if (mc_intr_0 & mc_intr_0_pgraph_pending_f())
			gk20a_gr_isr(g);
		if (mc_intr_0 & mc_intr_0_pfifo_pending_f())
			gk20a_fifo_isr(g);
		if (mc_intr_0 & mc_intr_0_pmu_pending_f())
			gk20a_pmu_isr(g);
		if (mc_intr_0 & mc_intr_0_priv_ring_pending_f())
			gk20a_priv_ring_isr(g);
		mc_intr_0 = gk20a_readl(g, mc_intr_0_r());
	}

	if (mc_intr_0)
		nvhost_dbg_info("leaving isr with interrupt pending 0x%08x",
				mc_intr_0);

	gk20a_writel(g, mc_intr_en_0_r(),
		mc_intr_en_0_inta_hardware_f());

	/* flush previous write */
	gk20a_readl(g, mc_intr_en_0_r());

	return IRQ_HANDLED;
}



static void gk20a_remove_support(struct platform_device *dev)
{
	struct gk20a *g = get_gk20a(dev);

	if (g->fifo.remove_support)
		g->fifo.remove_support(&g->fifo);

	if (g->sim.remove_support)
		g->sim.remove_support(&g->sim);

	if (g->irq_requested) {
		free_irq(gk20a_intr.start, g);
		g->irq_requested = false;
	}

	/* free mappings to registers, etc*/

	if (g->regs) {
		iounmap(g->regs);
		g->regs = 0;
	}
}

int nvhost_init_gk20a_support(struct platform_device *dev)
{
	int err = 0;
	struct gk20a *g = get_gk20a(dev);
	struct nvhost_device_data *pdata = nvhost_get_devdata(dev);

	g->regs = pdata->aperture[GK20A_BAR0_IORESOURCE_MEM];
	if (!g->regs) {
		dev_err(dev_from_gk20a(g), "failed to remap gk20a registers\n");
		err = -ENXIO;
		goto fail;
	}

	g->bar1 = pdata->aperture[GK20A_BAR1_IORESOURCE_MEM];
	if (!g->bar1) {
		dev_err(dev_from_gk20a(g), "failed to remap gk20a bar1\n");
		err = -ENXIO;
		goto fail;
	}

	err = request_threaded_irq(gk20a_intr.start,
			gk20a_intr_isr, gk20a_intr_thread,
			0, "gk20a", g);
	if (err) {
		dev_err(dev_from_gk20a(g), "failed to request stall interrupt irq @ %d\n",
			gk20a_intr.start);
		goto fail;
	}
	g->irq_requested = true;

	/* remove gk20a clamp in t124 soc register */
	writel(0, pmc + APBDEV_PMC_GPU_RG_CNTRL_0);

	gk20a_writel(g, mc_intr_en_1_r(),
		mc_intr_en_1_inta_disabled_f());

	gk20a_writel(g, mc_intr_en_0_r(),
		mc_intr_en_0_inta_hardware_f());

	err = gk20a_init_sim_support(dev);
	if (err)
		goto fail;

	err = gk20a_init_clk_support(g, false);
	if (err)
		goto fail;

	err = gk20a_init_mm_support(g, false);
	if (err)
		goto fail;

	err = gk20a_init_fifo_support(g, false);
	if (err)
		goto fail;

	err = gk20a_init_therm_support(g, false);
	if (err)
		goto fail;

	/* init_gr & init_pmu are deferred */

	g->remove_support = gk20a_remove_support;
	return 0;

 fail:
	gk20a_remove_support(dev);
	return err;
}

void nvhost_gk20a_init(struct platform_device *dev)
{
	struct gk20a *g = get_gk20a(dev);
	int err;

	BUG_ON(!g);
	nvhost_dbg_fn("");

	err = gk20a_init_gr_support(g, false);
	if (err)
		nvhost_err(&dev->dev, "failed init gk20a gr support\n");

	err = gk20a_init_pmu_support(g, false);
	if (err)
		nvhost_err(&dev->dev, "failed init gk20a pmu support\n");
}
static void nvhost_gk20a_deinit(struct platform_device *dev)
{

	struct gk20a *g = get_gk20a(dev);
	nvhost_dbg_fn("");

	if (g && g->remove_support)
		g->remove_support(dev);
	set_gk20a(dev, 0);
	kfree(g);
}


static void gk20a_free_hwctx(struct kref *ref)
{
	struct nvhost_hwctx *ctx = container_of(ref, struct nvhost_hwctx, ref);
	nvhost_dbg_fn("");

	gk20a_free_channel(ctx);
	kfree(ctx);
}


static struct nvhost_hwctx *gk20a_alloc_hwctx(struct nvhost_hwctx_handler *h,
					      struct nvhost_channel *ch)
{
	struct nvhost_hwctx *ctx;
	nvhost_dbg_fn("");

	/* it seems odd to be allocating a channel here but the
	 * t20/t30 notion of a channel is mapped on top of gk20a's
	 * channel.  this works because there is only one module
	 * under gk20a's host (gr).
	 */
	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return NULL;

	kref_init(&ctx->ref);
	ctx->h = h;
	ctx->channel = ch;

	return gk20a_open_channel(ch, ctx);
}

static void gk20a_get_hwctx(struct nvhost_hwctx *hwctx)
{
	nvhost_dbg_fn("");
	kref_get(&hwctx->ref);
}

static void gk20a_put_hwctx(struct nvhost_hwctx *hwctx)
{
	nvhost_dbg_fn("");
	kref_put(&hwctx->ref, gk20a_free_hwctx);
}

static void gk20a_save_push_hwctx(struct nvhost_hwctx *ctx, struct nvhost_cdma *cdma )
{
	nvhost_dbg_fn("");
}

static void gk20a_save_service_hwctx(struct nvhost_hwctx *ctx)
{
	nvhost_dbg_fn("");
}


static struct nvhost_hwctx_handler *
    nvhost_gk20a_alloc_hwctx_handler(u32 syncpt, u32 waitbase,
				     struct nvhost_channel *ch)
{

	struct nvhost_hwctx_handler *h;
	nvhost_dbg_fn("");

	h = kmalloc(sizeof(*h), GFP_KERNEL);
	if (!h)
		return NULL;

	h->alloc = gk20a_alloc_hwctx;
	h->get   = gk20a_get_hwctx;
	h->put   = gk20a_put_hwctx;
	h->save_push = gk20a_save_push_hwctx;
	h->save_service = gk20a_save_service_hwctx;

	return h;
}

static int __devinit gk20a_probe(struct platform_device *dev)
{
	int err;
	struct gk20a *gk20a;
	struct nvhost_device_data *pdata =
		(struct nvhost_device_data *)dev->dev.platform_data;

	nvhost_dbg_fn("");

	pdata->pdev = dev;
	platform_set_drvdata(dev, pdata);

	pdata->init                = nvhost_gk20a_init;
	pdata->deinit              = nvhost_gk20a_deinit;
	pdata->alloc_hwctx_handler = nvhost_gk20a_alloc_hwctx_handler;
	pdata->syncpt_base = 32; /*hack*/

	err = nvhost_client_device_get_resources(dev);
	if (err)
		return err;

	err = nvhost_client_device_init(dev);
	if (err) {
		nvhost_dbg_fn("failed to init client device for %s",
			      dev->name);
		return err;
	}

	err = nvhost_as_init_device(dev);
	if (err) {
		nvhost_dbg_fn("failed to init client address space"
			      " device for %s", dev->name);
		return err;
	}

	nvhost_dbg_fn("allocating gk20a support");
	gk20a = kzalloc(sizeof(struct gk20a), GFP_KERNEL);
	if (!gk20a) {
		dev_err(&dev->dev, "couldn't allocate gk20a support");
		err = -ENOMEM;
		goto fail;
	}

	set_gk20a(dev, gk20a);
	gk20a->dev = dev;
	gk20a->host = nvhost_get_host(dev);

	err = nvhost_init_gk20a_support(dev);

	if (err)
		goto fail;

	return 0;

 fail:
	dev_err(&dev->dev, "failed: %d", err);
	return err;
}

static int __exit gk20a_remove(struct platform_device *dev)
{
	/* Add clean-up */
	return 0;
}

#ifdef CONFIG_PM
static int gk20a_suspend(struct platform_device *dev, pm_message_t state)
{
	nvhost_dbg_fn("");
	return nvhost_client_device_suspend(dev);
}

static int gk20a_resume(struct platform_device *dev)
{
	nvhost_dbg_fn("");
	return 0;
}
#endif

static struct platform_driver gk20a_driver = {
	.probe = gk20a_probe,
	.remove = __exit_p(gk20a_remove),
#ifdef CONFIG_PM
	.suspend = gk20a_suspend,
	.resume = gk20a_resume,
#endif
	.driver = {
		.owner = THIS_MODULE,
		.name = "gk20a",
	}
};

static int __init gk20a_init(void)
{
	return platform_driver_register(&gk20a_driver);
}

static void __exit gk20a_exit(void)
{
	platform_driver_unregister(&gk20a_driver);
}

module_init(gk20a_init);
module_exit(gk20a_exit);