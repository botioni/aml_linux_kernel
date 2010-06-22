/***************************************************************************************
amlogic networks .....
add by zhouzhi 2008-8-18
 ***************************************************************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/io.h>
#include <linux/mii.h>
#include <asm/delay.h>
#include <mach/pinmux.h>
#include <linux/crc32.h>

#include "am_net8218.h"
//#include "asm/bsp.h"
// >0 for basic init and remove debug;
// >1 further setting debug;
// >2 for tx
// >3 for rx
static volatile int debug = 1;

static int running = 0;
static struct net_device *my_ndev = NULL;
static char DEFMAC[] = "\x00\x01\x23\xcd\xee\xaf";

#define DRV_NAME	"Amlogic Ethernet"
#define DRV_VERSION	"v2.0.0"
#define DRV_RELDATE	"2008-8-28"

MODULE_AUTHOR("rising_o<zhi_zhou@amlogic.com>");
MODULE_DESCRIPTION("Amlogic Ethernet Driver");
MODULE_LICENSE("Amlogic");
MODULE_VERSION(DRV_VERSION);

#define PERIPHS_SET_BITS(reg,val)	{	\
    	WRITE_PERIPHS_REG(reg,READ_PERIPHS_REG(reg) |(val));}
#define PERIPHS_CLEAR_BITS(reg,val)	{	\
	WRITE_PERIPHS_REG(reg,READ_PERIPHS_REG(reg) & ~(val));}

static void write_mac_addr(struct net_device *dev, char *macaddr);
static int ethernet_reset(struct net_device *dev);
static int mdio_read(struct net_device *dev, int phyid, int reg)
{
#define WR (1<<1)
#define MDCCLK (0x1) << 2	//our 130 MHz
#define BUSY 0x1

		struct am_net_private *priv = netdev_priv(dev);
		unsigned long busy = 0;
		unsigned long reg4;
		unsigned long val = 0;
		reg4 = phyid << 11 | reg << 6 | MDCCLK | BUSY;
		/*      
		do{ //waiting the phy is ready to write ...
		busy=IO_READ32(priv->base_addr+ETH_MAC_4_GMII_Addr);
		}while(busy&0x1);
		*/
		IO_WRITE32(reg4, priv->base_addr + ETH_MAC_4_GMII_Addr);
		do {			//waiting the phy is ready to write ...
			busy = IO_READ32(priv->base_addr + ETH_MAC_4_GMII_Addr);
		} while (busy & 0x1);
		val = IO_READ32(priv->base_addr + ETH_MAC_5_GMII_Data) & 0xffff;
		return val;
}

static void mdio_write(struct net_device *dev, int phyid, int reg, int val)
{

#define WR (1<<1)
#define MDCCLK (0x1) << 2	//our 130 MHz
#define BUSY 0x1

		struct am_net_private *priv = netdev_priv(dev);
		unsigned long busy = 0;
		unsigned long reg4;
		reg4 = phyid << 11 | reg << 6 | MDCCLK | WR | BUSY;
		IO_WRITE32(val, priv->base_addr + ETH_MAC_5_GMII_Data);
		do {			//waiting the phy is ready to write ...
			busy = IO_READ32(priv->base_addr + ETH_MAC_4_GMII_Addr);
		} while (busy & 0x1);
		IO_WRITE32(reg4, priv->base_addr + ETH_MAC_4_GMII_Addr);
		do {			//waiting the phy is ready to write ...
			busy = IO_READ32(priv->base_addr + ETH_MAC_4_GMII_Addr);
		} while (busy & 0x1);
}

static void dump(unsigned char *p, int len)
{
	int i, j;
	char s[20];
	for (i = 0; i < len; i += 16) {
		printk("%08x:", (unsigned int)p);
		for (j = 0; j < 16 && j < len - 0 * 16; j++) {
			s[j] = (p[j] > 15 && p[j] < 128) ? p[j] : '.';
			printk(" %02x", p[j]);
		}
		s[j] = 0;
		printk(" |%s|\n", s);
		p = p + 16;
	}
}

static int netdev_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	struct mii_ioctl_data *data = if_mii(rq);
	struct am_net_private *np = netdev_priv(dev);
	char addr[MAX_ADDR_LEN];
	if (debug > 0)
		printk("Ethernet Driver  ioctl (%x) \n", cmd);
	switch (cmd) {
	case SIOCGMIIPHY:	/* Get address of MII PHY in use. */
		data->phy_id =
		    ((struct am_net_private *)netdev_priv(dev))->phys[0] & 0x1f;
		/* Fall Through */

	case SIOCGMIIREG:	/* Read MII PHY register. */
		spin_lock_irq(&np->lock);
		data->val_out =
		    mdio_read(dev, data->phy_id & 0x1f, data->reg_num & 0x1f);
		spin_unlock_irq(&np->lock);
		return 0;

	case SIOCSMIIREG:	/* Write MII PHY register. */
		if (!capable(CAP_NET_ADMIN))
			return -EPERM;
		spin_lock_irq(&np->lock);
		mdio_write(dev, data->phy_id & 0x1f, data->reg_num & 0x1f,data->val_in);
		spin_unlock_irq(&np->lock);
		return 0;
	case SIOCSIFHWADDR:

		if (copy_from_user(&addr,
				   (void __user *)rq->ifr_hwaddr.sa_data,
				   MAX_ADDR_LEN)) {
			return -EFAULT;
		}
		if (debug > 0)
			printk("set mac addr to %02x:%02x:%02x:%02x:%02x:%02x\n",
			     addr[0], addr[1], addr[2], addr[3], addr[4],
			     addr[5]);
		spin_lock_irq(&np->lock);
		memcpy(dev->dev_addr, &addr, MAX_ADDR_LEN);
		write_mac_addr(dev, addr);
		spin_unlock_irq(&np->lock);
	default:
		if (debug > 0)
			printk("Ethernet Driver unknow ioctl (%x) \n", cmd);
		return -EOPNOTSUPP;
	}
	return 0;
}

int init_rxtx_rings(struct net_device *dev)
{
	struct am_net_private *np = netdev_priv(dev);
	int i;
#ifndef DMA_USE_SKB_BUF
	unsigned long tx = 0, rx = 0;
#endif
#ifdef DMA_USE_MALLOC_ADDR
	rx = (unsigned long)kmalloc((RX_RING_SIZE) * np->rx_buf_sz, GFP_KERNEL);
	if (rx == 0) {
		printk("error to alloc Rx  ring buf\n");
		return -1;
	}
	tx = (unsigned long)kmalloc((TX_RING_SIZE) * np->rx_buf_sz, GFP_KERNEL);
	if (tx == 0) {
		kfree((void *)rx);
		printk("error to alloc Tx  ring buf\n");
		return -1;
	}
#elif defined(DMA_USE_SKB_BUF)
	//not needed
#else
	tx = TX_BUF_ADDR;
	rx = RX_BUF_ADDR;
#endif

	/* Fill in the Rx buffers.  Handle allocation failure gracefully. */
	for (i = 0; i < RX_RING_SIZE; i++) {
#ifdef DMA_USE_SKB_BUF
		struct sk_buff *skb = dev_alloc_skb(np->rx_buf_sz);
		np->rx_ring[i].skb = skb;
		if (skb == NULL)
			break;
		skb_reserve(skb, 2);	/* 16 byte alignd for ip */
		skb->dev = dev;	/* Mark as being used by this device. */
		np->rx_ring[i].buf = (unsigned long)skb->data;
#else
		np->rx_ring[i].skb = NULL;
		np->rx_ring[i].buf = (rx + i * np->rx_buf_sz);	//(unsigned long )skb->data;
#endif
		np->rx_ring[i].buf_dma=dma_map_single(&dev->dev,(void *)np->rx_ring[i].buf,np->rx_buf_sz,DMA_FROM_DEVICE);
		np->rx_ring[i].count =(DescChain) | (np->rx_buf_sz & DescSize1Mask);
		np->rx_ring[i].status = (DescOwnByDma);
		np->rx_ring[i].next = &np->rx_ring[i + 1];

	}

	np->rx_ring[RX_RING_SIZE - 1].next = &np->rx_ring[0];
	/* Initialize the Tx descriptors */
	for (i = 0; i < TX_RING_SIZE; i++) {
#ifdef DMA_USE_SKB_BUF
		np->tx_ring[i].buf = 0;
#else
		np->tx_ring[i].buf = (tx + i * np->rx_buf_sz);
#endif
		np->tx_ring[i].status = 0;
		np->tx_ring[i].count =
		    (DescChain) | (np->rx_buf_sz & DescSize1Mask);
		np->tx_ring[i].next = &np->tx_ring[i + 1];
		np->tx_ring[i].skb = NULL;
	}
	np->tx_ring[TX_RING_SIZE - 1].next = &np->tx_ring[0];
	np->start_tx = &np->tx_ring[0];
	np->last_tx = NULL;
	np->last_rx = &np->rx_ring[RX_RING_SIZE - 1];

	return 0;
}

EXPORT_SYMBOL(init_rxtx_rings);
static int alloc_ringdesc(struct net_device *dev)
{
	struct am_net_private *np = netdev_priv(dev);

	np->rx_buf_sz = (dev->mtu <= 1500 ? PKT_BUF_SZ : dev->mtu + 32);

	/*np->rx_ring = kmalloc(sizeof(struct _rx_desc) * RX_RING_SIZE +
				sizeof(struct _tx_desc) * TX_RING_SIZE +
			      	CACHE_LINE, GFP_DMA); */
	np->rx_ring=dma_alloc_coherent(&dev->dev,
				sizeof(struct _rx_desc) * RX_RING_SIZE +CACHE_LINE,
				(dma_addr_t *)&np->rx_ring_dma,GFP_KERNEL);
	//np->rx_ring=0xc1001000;//apollo on chip sram

	if (!np->rx_ring)
		return -ENOMEM;

	if (!IS_CACHE_ALIGNED(np->rx_ring)) {
		printk("Error the alloc mem is not cache aligned(%p)\n",np->rx_ring);
	}
	printk("NET MDA descpter start addr=%p\n", np->rx_ring);

	np->tx_ring=dma_alloc_coherent(&dev->dev,
				sizeof(struct _tx_desc) * TX_RING_SIZE +CACHE_LINE,
				(dma_addr_t *)&np->tx_ring_dma,GFP_KERNEL);
	memset(np->rx_ring, 0,
	       sizeof(struct _rx_desc) * RX_RING_SIZE +
	       sizeof(struct _tx_desc) * TX_RING_SIZE);
	if (init_rxtx_rings(dev)) {
		printk("init rx tx ring failed!!\n");
		return -1;
	}
	//make sure all the data are write to memory
	return 0;
}

static int free_ringdesc(struct net_device *dev)
{
	struct am_net_private *np = netdev_priv(dev);
	int i;
	for (i = 0; i < RX_RING_SIZE; i++) {
		if (np->rx_ring[i].skb)
			dev_kfree_skb_any(np->rx_ring[i].skb);
		np->rx_ring[i].skb = NULL;
	}
	for (i = 0; i < TX_RING_SIZE; i++) {
		if (np->tx_ring[i].skb != NULL)
			dev_kfree_skb_any(np->tx_ring[i].skb);
		np->tx_ring[i].skb = NULL;
	}
	if (np->rx_ring)
		{
		dma_free_coherent(&dev->dev,
			sizeof(struct _rx_desc) * RX_RING_SIZE +CACHE_LINE,
			np->rx_ring,(dma_addr_t )np->rx_ring_dma);	// for apollo
		}
	np->tx_ring = NULL;
	if (np->tx_ring)
		{
		dma_free_coherent(&dev->dev,
			sizeof(struct _tx_desc) * RX_RING_SIZE +CACHE_LINE,
			np->tx_ring,(dma_addr_t )np->tx_ring_dma);	// for apollo
		}
	np->rx_ring = NULL;
	return 0;
}

static void netdev_timer(unsigned long data)
{
	struct net_device *dev = (struct net_device *)data;
	struct am_net_private *np = netdev_priv(dev);
	unsigned long ioaddr = np->base_addr;
	static int error_num = 0;
	int val;
	spin_lock_irq(&np->lock);
	val = mdio_read(dev, np->phys[0], MII_BMSR);
	spin_unlock_irq(&np->lock);
	if (debug > 2)
		printk(KERN_DEBUG "%s: Media selection timer tick, mac status %8.8x "
		       "MII SR %8.8x.\n", dev->name,
		       ioread32(ioaddr + ETH_DMA_5_Status), val);
	if (!(val & (BMSR_LSTATUS))) {	//unlink .....
		error_num++;
		if (error_num > 30) {
			error_num = 0;
			spin_lock_irq(&np->lock);
			val = (1 << 14) | (7 << 5) | np->phys[0];
			mdio_write(dev, np->phys[0], 18, val);
			// Auto negotiation restart 
			val = BMCR_ANENABLE | BMCR_ANRESTART;
			mdio_write(dev, np->phys[0], MII_BMCR, val);
			spin_unlock_irq(&np->lock);
		}
		np->timer.expires = jiffies + 2 * HZ;
		netif_stop_queue(dev);
		netif_carrier_off(dev);
		np->phy_set[0] = 0;
	} else {		//linked 
		spin_lock_irq(&np->lock);
		val = mdio_read(dev, np->phys[0], 31);
		spin_unlock_irq(&np->lock);
		if (np->phy_set[0] != val) {
			int tmp = 0;
			np->phy_set[0] = val;
			if ((val >> 4) & 1) {
				if (debug > 0)
					printk("duplex\n");
				//(*ETH_MAC_0_Configuration) |= 1<<11; // program mac
				tmp =
				IO_READ32(np->base_addr +ETH_MAC_0_Configuration);
				tmp |= 1 << 11;
				IO_WRITE32(tmp,np->base_addr +ETH_MAC_0_Configuration);
			} else {
				if (debug > 0)
					printk("half duplex\n");
				//(*ETH_MAC_0_Configuration) &= ~(1<<11) ; // program mac
				tmp =
				IO_READ32(np->base_addr +ETH_MAC_0_Configuration);
				tmp &= ~(1 << 11);
				IO_WRITE32(tmp, np->base_addr +ETH_MAC_0_Configuration);
			}
			if (val & (1 << 2)) {
				if (debug > 0)
					printk("10m\n");
				//(*ETH_MAC_0_Configuration) &= ~(1<<14); // program mac
				tmp =
				IO_READ32(np->base_addr +ETH_MAC_0_Configuration);
				tmp &= ~(1 << 14);
				IO_WRITE32(tmp,np->base_addr +ETH_MAC_0_Configuration);
				PERIPHS_CLEAR_BITS(HHI_ETH_CLK_CNTL, 1);
				PERIPHS_CLEAR_BITS(HHI_ETH_CLK_CNTL, (1 << 1));
				PERIPHS_SET_BITS(HHI_ETH_CLK_CNTL, 1);
			} else if (val & (1 << 3)) {
				if (debug > 0)
					printk("100m\n");
				//(*ETH_MAC_0_Configuration) |= 1<<14; // program mac
				tmp =
				IO_READ32(np->base_addr +ETH_MAC_0_Configuration);
				tmp |= 1 << 14;
				PERIPHS_CLEAR_BITS(HHI_ETH_CLK_CNTL, 1);
				IO_WRITE32(tmp,np->base_addr +ETH_MAC_0_Configuration);
				PERIPHS_SET_BITS(HHI_ETH_CLK_CNTL, (1 << 1));
				PERIPHS_SET_BITS(HHI_ETH_CLK_CNTL, 1);

			}
		}
		error_num = 0;
		netif_carrier_on(dev);
		netif_start_queue(dev);
		np->timer.expires = jiffies + 10 * HZ;
	}
	add_timer(&np->timer);
}

static inline int update_status(struct net_device *dev, unsigned long status,
				unsigned long mask)
{
	struct am_net_private *np = netdev_priv(dev);
	int need_reset = 0;
	int need_rx_restart = 0;
	int res = 0;
	if (status & NOR_INTR_EN)	//Normal Interrupts Process
	{
		if (status & TX_INTR_EN)	//Transmit Interrupt Process
		{
			IO_WRITE32((1 << 0 | 1 << 16),np->base_addr + ETH_DMA_5_Status);
			res |= 1;
		}
		if (status & RX_INTR_EN)	//Receive Interrupt Process
		{
			IO_WRITE32((1 << 6 | 1 << 16),np->base_addr + ETH_DMA_5_Status);
			res |= 2;
		}
		if (status & EARLY_RX_INTR_EN) {
			IO_WRITE32((EARLY_RX_INTR_EN | NOR_INTR_EN),np->base_addr + ETH_DMA_5_Status);
		}
		if (status & TX_BUF_UN_EN) {
			IO_WRITE32((1 << 2 | 1 << 16),np->base_addr + ETH_DMA_5_Status);
			res |= 1;
			//this error will cleard in start tx...
			if (debug > 1)
				printk(KERN_WARNING "[" DRV_NAME "]" "Tx bufer unenable\n");
		}
	} else if (status & ANOR_INTR_EN)	//Abnormal Interrupts Process
	{
		if (status & RX_BUF_UN) {
			IO_WRITE32((RX_BUF_UN | ANOR_INTR_EN),np->base_addr + ETH_DMA_5_Status);
			np->stats.rx_over_errors++;
			need_rx_restart++;
			res |= 2;
			//printk(KERN_WARNING DRV_NAME "Receive Buffer Unavailable\n");
			if (debug > 1)
				printk(KERN_WARNING "[" DRV_NAME "]" "Rx bufer unenable\n");
		}
		if (status & RX_STOP_EN) {
			IO_WRITE32((RX_STOP_EN | ANOR_INTR_EN),
				   np->base_addr + ETH_DMA_5_Status);
			need_rx_restart++;
			res |= 2;
		}
		if (status & RX_WATCH_TIMEOUT) {
			IO_WRITE32((RX_WATCH_TIMEOUT | ANOR_INTR_EN),
				   np->base_addr + ETH_DMA_5_Status);
			need_rx_restart++;
		}
		if (status & FATAL_BUS_ERROR) {
			IO_WRITE32((FATAL_BUS_ERROR | ANOR_INTR_EN),
				   np->base_addr + ETH_DMA_5_Status);
			need_reset++;
 			printk(KERN_WARNING "[" DRV_NAME "]" "fatal bus error\n");
		}
		if (status & EARLY_TX_INTR_EN) {
			IO_WRITE32((EARLY_TX_INTR_EN | ANOR_INTR_EN),
				   np->base_addr + ETH_DMA_5_Status);
		}
		if (status & TX_STOP_EN) {
			IO_WRITE32((TX_STOP_EN | ANOR_INTR_EN),
				   np->base_addr + ETH_DMA_5_Status);
			res |= 1;
		}
		if (status & TX_JABBER_TIMEOUT) {
			IO_WRITE32((TX_JABBER_TIMEOUT | ANOR_INTR_EN),
				   np->base_addr + ETH_DMA_5_Status);
			printk(KERN_WARNING "[" DRV_NAME "]" "tx jabber timeout\n");
			np->first_tx = 1;
		}
		if (status & RX_FIFO_OVER) {
			IO_WRITE32((RX_FIFO_OVER | ANOR_INTR_EN),
				   np->base_addr + ETH_DMA_5_Status);
			np->stats.rx_fifo_errors++;
			need_rx_restart++;
			res |= 2;
			printk(KERN_WARNING "[" DRV_NAME "]" "Rx fifo over\n");
		}
		if (status & TX_UNDERFLOW) {
			IO_WRITE32((TX_UNDERFLOW | ANOR_INTR_EN),
				   np->base_addr + ETH_DMA_5_Status);
			printk(KERN_WARNING "[" DRV_NAME "]" "Tx underflow\n");
			np->first_tx = 1;
			res |= 1;
		}
	}

	if (need_reset) {
		printk(KERN_WARNING DRV_NAME "system reset\n");
		free_ringdesc(dev);
		ethernet_reset(dev);
	} else if (need_rx_restart) {
		IO_WRITE32(1, np->base_addr + ETH_DMA_2_Re_Poll_Demand);
	}
	return res;
}

static void inline print_rx_error_log(unsigned long status)
{

	if (status & DescRxTruncated) {
		printk(KERN_WARNING "Descriptor Error desc-mask[%d]\n",
		       DescRxTruncated);
	}
	if (status & DescSAFilterFail) {
		printk(KERN_WARNING
		       "Source Address Filter Fail rx desc-mask[%d]\n",
		       DescSAFilterFail);
	}
	if (status & DescRxLengthError) {
		printk(KERN_WARNING "Length Error rx desc-mask[%d]\n",
		       DescRxLengthError);
	}
	if (status & DescRxIPChecksumErr) {
		printk(KERN_WARNING "TCP checksum Error rx desc-mask[%d]\n",
		       DescRxLengthError);
	}
	if (status & DescRxTCPChecksumErr) {
		printk(KERN_WARNING "TCP checksum Error rx desc-mask[%d]\n",
		       DescRxLengthError);
	}
	if (status & DescRxDamaged) {
		printk(KERN_WARNING "Overflow Error rx desc-mask[%d]\n",
		       DescRxDamaged);
	}
	if (status & DescRxMiiError) {
		printk(KERN_WARNING "Receive Error rx desc-mask[%d]\n",
		       DescRxMiiError);
	}
	if (status & DescRxDribbling) {
		printk(KERN_WARNING "Dribble Bit Error rx desc-mask[%d]\n",
		       DescRxDribbling);
	}
	if (status & DescRxCrc) {
		printk(KERN_WARNING "CE: CRC Errorrx desc-mask[%d]\n",
		       DescRxCrc);
	}
}

/**********************
we do the basic rx  tx operation irq;
FIXME:on SMP system..

************************/
void net_tasklet(unsigned long dev_instance)
{
	struct net_device *dev = (struct net_device *)dev_instance;
	struct am_net_private *np = netdev_priv(dev);
	int len;
	int result;
	unsigned long flags,status,mask;

#ifndef DMA_USE_SKB_BUF
	struct sk_buff *skb = NULL;
#endif
	if (!running)
		goto release;
        status = IO_READ32(np->base_addr + ETH_DMA_5_Status);
        result= update_status(dev, status, mask);
	if (result & 1) {
		struct _tx_desc *c_tx, *tx = NULL;

		c_tx =(void *)IO_READ32(np->base_addr +ETH_DMA_18_Curr_Host_Tr_Descriptor);
		tx = np->start_tx;
		while (tx != NULL && tx != c_tx && !(tx->status & DescOwnByDma)) {
#ifdef DMA_USE_SKB_BUF
			spin_lock_irqsave(&np->lock, flags);
			if (tx->skb != NULL) {
				//clear to next send;
				if (np->tx_full) {
					netif_wake_queue(dev);
					np->tx_full = 0;
				}
				dev_kfree_skb_any(tx->skb);
				tx->skb = NULL;
				tx->buf = 0;
				tx->status = 0;
			} else
				break;
			spin_unlock_irqrestore(&np->lock, flags);
#else
			tx->status = 0;
			if (np->tx_full) {
				netif_wake_queue(dev);
				np->tx_full = 0;
			}
#endif
			tx = tx->next;
		}
		np->start_tx = tx;
		//data tx end... todo 
	}
	if (result & 2) {
		//data  rx; 
		struct _rx_desc *c_rx, *rx = NULL;
		c_rx =
		    (void *)IO_READ32(np->base_addr +ETH_DMA_19_Curr_Host_Re_Descriptor);
		rx = np->last_rx->next;
		while (rx != NULL) {
			//if(rx->status !=IO_READ32(&rx->status))
			//      printk("error of D-chche!\n");
			if (!(rx->status & (DescOwnByDma))) {
				int ip_summed = CHECKSUM_UNNECESSARY;
				len = (rx->status & DescFrameLengthMask) >>DescFrameLengthShift;
				if (unlikely(len < 18 || len > np->rx_buf_sz)) {	//here is fatal error we drop it ;
					np->stats.rx_dropped++;
					np->stats.rx_errors++;
					goto to_next;
				}
				if (unlikely(rx->status & (DescError))) {	//here is not often occur
					print_rx_error_log(rx->status);
					//rx->status=DescOwnByDma;
					if ((rx->status & DescRxIPChecksumErr) || (rx->status & DescRxTCPChecksumErr)) {	//maybe checksum engine's problem;
						//we set the NONE for ip/tcp need check it again
						ip_summed = CHECKSUM_NONE;
					} else {
						np->stats.rx_dropped++;
						np->stats.rx_errors++;
						goto to_next;
					}
				}
				len = len - 4;	//clear the crc       
#ifdef DMA_USE_SKB_BUF
				if (IS_ERR(rx->skb)) {
					printk("NET skb pointer error\n");
					break;
				}
				skb_put(rx->skb, len);
				rx->skb->dev = dev;
				rx->skb->protocol =
				    eth_type_trans(rx->skb, dev);
				/*we have checked in hardware;
				   we not need check again */
				rx->skb->ip_summed = ip_summed;
				
				netif_rx(rx->skb);
#else
				skb = dev_alloc_skb(len + 4);
				if (skb == NULL) {
					np->stats.rx_dropped++;
					printk("error to alloc skb\n");
					break;
				}
				skb_reserve(skb, 2);
				skb_put(skb, len);
				memcpy(skb->data, (void *)rx->buf, len);
				skb->dev = dev;
				skb->protocol = eth_type_trans(skb, dev);
				skb->ip_summed = ip_summed;
				netif_rx(skb);
#endif
				dev->last_rx = jiffies;
				np->stats.rx_packets++;
				np->stats.rx_bytes += len;

				//*/
				//printk("receive data len=%d\n",len);
				//dump((unsigned char *)rx->buf,len);

				//reset the rx_ring to receive 
				///

			      to_next:
#ifdef DMA_USE_SKB_BUF
				rx->skb = dev_alloc_skb(np->rx_buf_sz + 4);
				if (IS_ERR(rx->skb)) {
					printk(KERN_ERR
					       "error to alloc the skb\n");
					rx->buf = 0;
					rx->status = 0;
					rx->count = 0;
					np->last_rx = rx;
					break;
				}
				skb_reserve(rx->skb, 2);
				rx->buf = (unsigned long)rx->skb->data;
#endif
				rx->buf_dma=dma_map_single(&dev->dev,(void *)rx->buf, (unsigned long)np->rx_buf_sz,DMA_FROM_DEVICE);	//invalidate for next  dma in;
				rx->count =(DescChain) | (np->rx_buf_sz &DescSize1Mask);
				rx->status = DescOwnByDma;
				np->last_rx = rx;
				rx = rx->next;
			} else {
				break;
			}

		}
	}
release:
	IO_WRITE32(np->irq_mask, (np->base_addr + ETH_DMA_7_Interrupt_Enable));
}

static irqreturn_t intr_handler(int irq, void *dev_instance)
{
	struct net_device *dev = (struct net_device *)dev_instance;
	struct am_net_private *np = netdev_priv(dev);
	IO_WRITE32(0, (np->base_addr + ETH_DMA_7_Interrupt_Enable));//disable irq
	tasklet_schedule(&np->rx_tasklet);
	return IRQ_HANDLED;
}

static int phy_reset(struct net_device *ndev)
{
	struct am_net_private *np = netdev_priv(ndev);
	unsigned long val;
	int k;

	//mac reset ...
	IO_WRITE32(1, np->base_addr + ETH_DMA_0_Bus_Mode);
	//waiting mac reset...
	for (k = 0;
	     (IO_READ32(np->base_addr + ETH_DMA_0_Bus_Mode) & 1) && k < 1000;
	     k++)
		udelay(1);
	if (k >= 1000) {
		printk("error to reset mac!\n");
		goto error_reset;
	}
	//set for RMII mode;
	val = (1 << 14) | (7 << 5) | np->phys[0];
	mdio_write(ndev, np->phys[0], 18, val);
	val = BMCR_RESET;
	mdio_write(ndev, np->phys[0], MII_BMCR, val);
	//waiting to phy reset ok....
	for (k = 0; (mdio_read(ndev, np->phys[0], MII_BMCR)) & (BMCR_RESET)
	     && k < 1000; k++) {
		udelay(1);
	}
	if (k >= 1000) {
		printk("error to reset phy!\n");
		goto error_reset;
	}
	// mode = 111; turn on auto-neg mode (previously was power-saving)
	val = (1 << 14) | (7 << 5) | np->phys[0];
	mdio_write(ndev, np->phys[0], 18, val);
	// Auto negotiation restart 
	val = BMCR_ANENABLE | BMCR_ANRESTART;
	mdio_write(ndev, np->phys[0], MII_BMCR, val);
	if (debug > 1)
		printk("starting to auto negotiation!\n");

	//(*ETH_DMA_0_Bus_Mode) = 0x00100800;
	IO_WRITE32(0x00100800, np->base_addr + ETH_DMA_0_Bus_Mode);

	/*
	   val=*((unsigned short *)&ndev->dev_addr[4]);
	   IO_WRITE32(val,np->base_addr+ETH_MAC_Addr0_High);
	   val=*((unsigned long *)ndev->dev_addr);
	   IO_WRITE32(val,np->base_addr+ETH_MAC_Addr0_Low);
	 */
	write_mac_addr(ndev, ndev->dev_addr);

	val = 0xc80c |		//8<<8 | 8<<17; //tx and rx all 8bit mode;
	    	1 << 10;		//checksum offload enabled
	IO_WRITE32(val, np->base_addr + ETH_MAC_0_Configuration);

	val = 1 << 4;/*receive all muticast*/
	//| 1 << 31;	//receive all the data 
	IO_WRITE32(val, np->base_addr + ETH_MAC_1_Frame_Filter);

	IO_WRITE32((unsigned long)&np->rx_ring[0],(np->base_addr + ETH_DMA_3_Re_Descriptor_List_Addr));
	IO_WRITE32((unsigned long)&np->tx_ring[0],(np->base_addr + ETH_DMA_4_Tr_Descriptor_List_Addr));
	IO_WRITE32(np->irq_mask, (np->base_addr + ETH_DMA_7_Interrupt_Enable));
	IO_WRITE32((0), (np->base_addr + ETH_MAC_Interrupt_Mask));
	val = (0x00000002 | 7 << 14 | 1 << 25 | 1 << 8 | 1 << 26 | 1 << 21);
	////1<<21 is Transmit Store and Forward used for tcp/ip checksum insert
	IO_WRITE32(val, (np->base_addr + ETH_DMA_6_Operation_Mode));
	np->phy_set[0] = 0;	//make sure reset the phy speed
	return 0;
      error_reset:

	return -1;
}

static int ethernet_reset(struct net_device *dev)
{
	struct am_net_private *np = netdev_priv(dev);
	int res;

	res = alloc_ringdesc(dev);
	if (res != 0) {
		printk(KERN_INFO "can't alloc ring desc!err=%d\n", res);
		goto out_err;
	}
	res = phy_reset(dev);
	if (res != 0) {
		printk(KERN_INFO "can't reset ethernet phy!err=%d\n", res);
		goto out_err;
	}
	np->first_tx = 1;

      out_err:
	return res;
}

static int netdev_open(struct net_device *dev)
{
	struct am_net_private *np = netdev_priv(dev);
	unsigned long flags;
	int res;
	if (running)
		return 0;
	spin_lock_irqsave(&np->lock, flags);
	res = ethernet_reset(dev);
	spin_unlock_irqrestore(&np->lock, flags);
	if (res != 0) {
		printk(KERN_INFO "ethernet_reset err=%d\n", res);
		goto out_err;
	}
	//netif_device_detach(dev);
	res = request_irq(dev->irq, &intr_handler, IRQF_SHARED, dev->name, dev);
	if (res) {
		printk(KERN_ERR "%s: request_irq error %d.,err=%d\n",
		       dev->name, dev->irq, res);
		goto out_err;
	}

	if (debug > 0)
		printk(KERN_DEBUG "%s: opened (irq %d).\n",
		       				dev->name, dev->irq);
	//enable_irq(dev->irq);
	/* Set the timer to check for link beat. */
	init_timer(&np->timer);
	np->timer.expires = jiffies + 1 * HZ;
	np->timer.data = (unsigned long)dev;
	np->timer.function = &netdev_timer;	/* timer handler */
	add_timer(&np->timer);
	running = 1;
	return 0;
      out_err:
	running = 0;
	return -EIO;
}

static int netdev_close(struct net_device *dev)
{
	struct am_net_private *np = netdev_priv(dev);
	if (!running)
		return 0;
	running = 0;
	IO_WRITE32(0, np->base_addr + ETH_DMA_7_Interrupt_Enable);
	//netif_device_detach(dev);
	disable_irq(dev->irq);
	netif_carrier_off(dev);
	netif_stop_queue(dev);
	spin_lock_irq(&np->lock);
	free_ringdesc(dev);
	spin_unlock_irq(&np->lock);
	free_irq(dev->irq, dev);
	del_timer_sync(&np->timer);
//      free_rxtx_rings(np);
//      free_ringdesc(np); 
//      PERIPHS_CLEAR_BITS(ETH_PLL_CNTL,1);//disable clk        
	if (debug > 0)
		printk(KERN_DEBUG "%s: closed\n", dev->name);

	return 0;
}

static int start_tx(struct sk_buff *skb, struct net_device *dev)
{
	struct am_net_private *np = netdev_priv(dev);
	int tmp;
	struct _tx_desc *tx;
	unsigned long flags;
	dev->trans_start = jiffies;

	if (!running)
		return -1;
	if (debug > 2) {
		printk(KERN_DEBUG "%s: Transmit frame queued\n", dev->name);
	}
	spin_lock_irqsave(&np->lock, flags);
	if (np->last_tx != NULL)
		tx = np->last_tx->next;
	else
		tx = &np->tx_ring[0];
	if (tx->status & DescOwnByDma) {
		spin_unlock_irqrestore(&np->lock, flags);
		printk("tx queue is full \n");
		goto err;
	}
#ifdef DMA_USE_SKB_BUF
	if (tx->skb != NULL) {
		dev_kfree_skb_any(tx->skb);
	}
	tx->skb = skb;
	tx->buf = (unsigned long)skb->data;
#else
	memcpy((void *)tx->buf, skb->data, skb->len);
#endif
	tx->buf_dma=dma_map_single(&dev->dev,(void *)tx->buf,(unsigned long)(skb->len),DMA_TO_DEVICE);
	tx->count = ((skb->len << DescSize1Shift) & DescSize1Mask) | DescTxFirst | DescTxLast | DescTxIntEnable | DescChain;	//|2<<27; (1<<25, ring end)
	tx->status = DescOwnByDma;
	if (skb->ip_summed == CHECKSUM_PARTIAL) {
		tx->count |= 0x3 << 27;	//add hw check sum;
	}
	np->last_tx = tx;
	np->stats.tx_packets++;
	np->stats.tx_bytes += skb->len;
#ifndef DMA_USE_SKB_BUF
	dev_kfree_skb_any(skb);
#endif
	if (np->first_tx) {
		np->first_tx = 0;
		tmp = IO_READ32(np->base_addr + ETH_DMA_6_Operation_Mode);
		tmp |= (7 << 14) | (1 << 13);
		IO_WRITE32(tmp, np->base_addr + ETH_DMA_6_Operation_Mode);
	} else {
		//ETH_DMA_1_Tr_Poll_Demand
		IO_WRITE32(1, np->base_addr + ETH_DMA_1_Tr_Poll_Demand);
	}
	spin_unlock_irqrestore(&np->lock, flags);
	return 0;
      err:
	np->tx_full = 1;
	np->stats.tx_dropped++;
	netif_stop_queue(dev);
	return -1;
}

static struct net_device_stats *get_stats(struct net_device *dev)
{
	struct am_net_private *np = netdev_priv(dev);

	return &np->stats;
}

static void tx_timeout(struct net_device *dev)
{
	struct am_net_private *np = netdev_priv(dev);
	int val;
	//FIXME
	spin_lock_irq(&np->lock);
	val = mdio_read(dev, np->phys[0], MII_BMSR);
	spin_unlock_irq(&np->lock);
	if (!(val & (BMSR_LSTATUS))) {	//unlink .....
		netif_stop_queue(dev);
		netif_carrier_off(dev);
	} else {
		netif_carrier_on(dev);
		netif_wake_queue(dev);
		dev->trans_start = jiffies;
		np->stats.tx_errors++;
	}
	return;
}

static void write_mac_addr(struct net_device *dev, char *macaddr)
{
	struct am_net_private *np = netdev_priv(dev);
	unsigned int val;
	val = *((unsigned short *)&macaddr[4]);
	IO_WRITE32(val, np->base_addr + ETH_MAC_Addr0_High);
	val = *((unsigned long *)macaddr);
	IO_WRITE32(val, np->base_addr + ETH_MAC_Addr0_Low);
	printk("write mac add to:");
	dump(macaddr, 6);
}

static unsigned char inline chartonum(char c)
{
	if (c >= '0' && c <= '9')
		return c - '0';
	if (c >= 'A' && c <= 'F')
		return (c - 'A')+10;
	if (c >= 'a' && c <= 'f')
		return (c - 'a')+10;
	return 0;

}

static void config_mac_addr(struct net_device *dev)
{
	//Need fix..mac addr is read from eeprom..
	memcpy(dev->dev_addr, DEFMAC, 6);
	write_mac_addr(dev, dev->dev_addr);
}

static int __init mac_addr_set(char *line)
{
	unsigned char mac[6];
	int i = 0;
	for (i = 0; i < 6 && line[0] != '\0' && line[1] != '\0'; i++) {
		mac[i] = chartonum(line[0]) << 4 | chartonum(line[1]);
		line += 3;
	}
	memcpy(DEFMAC, mac, 6);
	return 1;
}

__setup("mac=", mac_addr_set);


static inline int phy_mc_hash(__u8 *addr)
{
	return (bitrev32(~crc32_le(~0, addr,ETH_ALEN))>>26);
}

static void set_multicast_list(struct net_device *dev)
{
	struct am_net_private *np = netdev_priv(dev);
	u32  tmp;
	if ((dev->flags & IFF_PROMISC)) {
		tmp=IO_READ32(np->base_addr + ETH_MAC_1_Frame_Filter);
		tmp|=1;
		IO_WRITE32(tmp, np->base_addr + ETH_MAC_1_Frame_Filter);//promisc module
		printk("ether enter promisc module\n");
	}
	else
	{
		tmp=IO_READ32(np->base_addr + ETH_MAC_1_Frame_Filter);
		tmp&=~1;
		IO_WRITE32(tmp, np->base_addr + ETH_MAC_1_Frame_Filter);//live promisc
		printk("ether live promisc module\n");
	}
	if ((dev->flags & IFF_ALLMULTI) ) {
		tmp=IO_READ32(np->base_addr + ETH_MAC_1_Frame_Filter);
		tmp|=(1<<4);
		IO_WRITE32(tmp, np->base_addr + ETH_MAC_1_Frame_Filter);//all muticast
		printk("ether enter all multicast module\n");
	}
	else
	{
		tmp=IO_READ32(np->base_addr + ETH_MAC_1_Frame_Filter);
		tmp&=(1<<4);
		IO_WRITE32(tmp, np->base_addr + ETH_MAC_1_Frame_Filter);//live all muticast
		printk("ether live all muticast module\n");
	}
	
	if (dev->mc_count > 0)
	{
		int cnt=dev->mc_count;
		u32 hash[2];
		struct dev_mc_list	*addr_list;
		u32 hash_id;
		char * addr;
		hash[0]=0;
		hash[1]=0;
		printk("changed the Multicast,mcount=%d\n",dev->mc_count);
		for (addr_list = dev->mc_list; cnt && addr_list != NULL; addr_list = addr_list->next, cnt--) {
			addr=addr_list->dmi_addr;
			hash_id=phy_mc_hash(addr);
			///*
			printk("add mac address:%02x:%02x:%02x:%02x:%02x:%02x,bit=%d\n",
				addr[0],addr[1],addr[2],addr[3],addr[4],addr[5],
				hash_id);
			//*/
			//set_bit(hash_id,hash);
			if(hash_id>31)
				hash[1]|=1<<(hash_id-32);
			else
				hash[0]|=1<<hash_id;
		}
		printk("set hash low=%x,high=%x\n",hash[0],hash[1]);
		IO_WRITE32(hash[1], np->base_addr + ETH_MAC_2_Hash_Table_High);
		IO_WRITE32(hash[0], np->base_addr + ETH_MAC_3_Hash_Table_Low);
		tmp=IO_READ32(np->base_addr + ETH_MAC_1_Frame_Filter);
		tmp= (1<<2) | 	//hash filter 
			0;
		printk("changed the filter setting to :%x\n",tmp);
		IO_WRITE32(tmp, np->base_addr + ETH_MAC_1_Frame_Filter);//hash muticast
	}
}

static const struct net_device_ops am_netdev_ops = {
	.ndo_open		= netdev_open,
	.ndo_stop		= netdev_close,
	.ndo_start_xmit 	= start_tx,
	.ndo_tx_timeout		= tx_timeout,
	.ndo_set_multicast_list = set_multicast_list,
	.ndo_do_ioctl		=netdev_ioctl,
	.ndo_get_stats		=get_stats
	//.ndo_change_mtu		= eth_change_mtu,
	//.ndo_set_mac_address 	= eth_mac_addr,
	//.ndo_validate_addr	= eth_validate_addr,
};

static int setup_net_device(struct net_device *dev)
{
	struct am_net_private *np = netdev_priv(dev);
	int res = 0;
	dev->features = NETIF_F_GEN_CSUM;
	dev->netdev_ops=&am_netdev_ops;
	dev->ethtool_ops = NULL;	// &netdev_ethtool_ops;
	dev->watchdog_timeo = TX_TIMEOUT;
	np->irq_mask= (1 << 16) |       //NIE: Normal Interrupt Summary Enable                                                                     
            	(1 << 15) |          //abnormal int summary                                                                                
            	(1 << 6) |          //Receive Interrupt Enable                                                                                 
            	(1 << 2) |          //Transmit Buffer Unavailable Enable                                                                       
            	(1 << 3) |          //TJT: Transmit Jabber Timeout                                                                             
            	(1 << 4) |          //OVF: Receive Overflow                                                                                    
            	(1 << 5) |          //UNF: Transmit Underflow                                                                                  
            	(1 << 7) |          //7 RU: Receive Buffer Unavailable                                                                         
            	(1 << 8) |          //RPS: Receive Process Stopped                                                                             
            	(1 << 13) |          //13 FBI: Fatal Bus Error Interrupt                                                                        
         	 (1) | 		//tx interrupt
	   	0; 
	config_mac_addr(dev);
	dev_alloc_name(dev, "eth%d");
	memset(&np->stats, 0, sizeof(np->stats));
	return res;
}

/***
I think this must be init at the system start ...
//zhouzhi
***/

static void bank_io_init(struct net_device *ndev)
{
	int chip=0;
	switch (chip) {
		default:
		///PERIPHS_SET_BITS(PREG_PIN_MUX_REG8, 0x3ff << 1);
		
			set_mio_mux(8,(0x3ff << 1));
		#if 0	
			PERIPHS_CLEAR_BITS(PREG_NDMA_AES_CONTROL, (1 << 12));
			PERIPHS_CLEAR_BITS(PREG_GPIOC_OUTLVL, (0x1 << 26));
			PERIPHS_CLEAR_BITS(PREG_GPIOC_OE, (0x1 << 26));
			udelay(100);	//waiting reset end;
			PERIPHS_SET_BITS(PREG_GPIOC_OUTLVL, (0x1 << 26));
			udelay(10);
		#endif	
		break;
		;
	}
	PERIPHS_CLEAR_BITS(HHI_ETH_CLK_CNTL, 1);	//disable clk                                
	PERIPHS_CLEAR_BITS(HHI_ETH_CLK_CNTL, (1 << 0 | 1 << 2 | 1 << 3));
	PERIPHS_SET_BITS(HHI_ETH_CLK_CNTL, (1 << 1));
	PERIPHS_SET_BITS(HHI_ETH_CLK_CNTL, (1 << 0));
	udelay(100);

}

static int probe_init(struct net_device *ndev)
{
	int phy = 0;
	int phy_idx = 0;
	int found = 0;
	int res;
	struct am_net_private *priv = netdev_priv(ndev);
	ndev->base_addr = (unsigned long)(ETHBASE);
	ndev->irq = ETH_INTERRUPT;
	spin_lock_init(&priv->lock);
	priv->mii_if.dev = ndev;
	priv->mii_if.mdio_read = mdio_read;
	priv->mii_if.mdio_write = mdio_write;
	priv->base_addr = ndev->base_addr;
	if (debug > 0)
		printk("addr is %x\n", (unsigned int)ndev->base_addr);

	bank_io_init(ndev);
	for (phy = 0; phy < 32 && phy_idx < MII_CNT; phy++) {
		int mii_status = mdio_read(ndev, phy, MII_BMSR);
		if (mii_status != 0xffff && mii_status != 0x0000) {
			priv->phys[phy_idx++] = phy;
			priv->mii_if.advertising =
			    mdio_read(ndev, phy, MII_ADVERTISE);
			priv->mii =
			    (mdio_read(ndev, phy, MII_PHYSID1) << 16) +
			    mdio_read(ndev, phy, MII_PHYSID2);
			if (debug > 0)
				printk(KERN_INFO
				       "%s: MII PHY %8.8xh found at address %d, status "
				       "0x%4.4x advertising %4.4x.\n", DRV_NAME,
				       priv->mii, phy, mii_status,
				       priv->mii_if.advertising);
			found++;
		}
	}
	if (!found) {
		printk("can't find any mii phy device !\n");
		res = -EIO;
		goto error0;
	}
	mdio_write(ndev, priv->phys[0], 18, priv->phys[0] | (1 << 14 | 7 << 5));
	res = setup_net_device(ndev);
	if (res != 0) {
		printk("setup net device error !\n");
		res = -EIO;
		goto error0;
	}

	res = register_netdev(ndev);
	if (res != 0) {
		printk("can't register net  device !\n");
		res = -EBUSY;
		goto error0;
	}
	tasklet_init(&priv->rx_tasklet, net_tasklet, (unsigned long)ndev);
	return 0;
//error1:
//      unregister_netdev(ndev);
      error0:

	return res;
}

static int __init am_net_init(void)
{
	int res;
	printk(DRV_NAME "init(dbg[%p])\n", (&debug));
	my_ndev = alloc_etherdev(sizeof(struct am_net_private));
	if (my_ndev == NULL) {
		printk(DRV_NAME "ndev alloc failed!!\n");
		return -ENOMEM;
	}
	res = probe_init(my_ndev);
	if (res != 0)
		free_netdev(my_ndev);
	return res;
}

static void am_net_free(struct net_device *ndev)
{
	//struct am_net_private *np=netdev_priv(ndev);
	netdev_close(ndev);
	unregister_netdev(ndev);
}

static void __exit am_net_exit(void)
{
	printk(DRV_NAME "exit\n");
	am_net_free(my_ndev);
	free_netdev(my_ndev);
	return;
}

module_init(am_net_init);
module_exit(am_net_exit);
