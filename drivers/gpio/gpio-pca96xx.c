

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/i2c/pcf857x.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/regmap.h>
#include "gpiolib.h"


#define PCA96XX_INFO(fmt, arg...) printk(KERN_INFO "PCA96xx: " fmt "\n" , ## arg)
#define PCA96XX_ERR(fmt, arg...)  printk(KERN_ERR "%s: " fmt "\n" , __func__ , ## arg)
#define PCA96XX_DBG(fmt, arg...)  pr_debug("%s: " fmt "\n" , __func__ , ## arg)


#define NR_GPIO      16
#define BANK_SIZE    8
#define PCA96xx_LBL  "pca96xx"
#define CAN_SLEEP    0

#define DIR_IN     1u
#define DIR_OUT    0u

#define GPIO1      0x01
#define GPIO2      0x02
#define GPIO3      0x04
#define GPIO4      0x08
#define GPIO5      0x0F
#define GPIO6      0x20
#define GPIO7      0x40
#define GPIO8      0x80
#define GPIO_ALL   0xFF


#define REG_INPORT0     0x0
#define REG_INPORT1     0x1
#define REG_OUTPORT0    0x2
#define REG_OUTPORT1    0x3
#define REG_POL_INV_P0  0x4
#define REG_POL_INV_P1  0x5
#define REG_CONF_PORT0  0x6
#define REG_CONF_PORT1  0x7



typedef struct pca96xx_data {
	struct gpio_chip	gpio_chip;

	struct i2c_client	*client;
	struct mutex		lock;
	struct mutex        lock_irq;

	uint8_t             cache[8];
	struct irq_domain   *irq_domain;
	int                 irq;

	int                 irq_used[NR_GPIO];
	int                 irq_fall[NR_GPIO];
	int                 irq_rise[NR_GPIO];

	struct regmap	    *regmap;
}pca96xx_t;

static struct lock_class_key gpio_lock_class;

/*  __________________________________________________________________________
 * |                                                                          |
 * |                              REGISTER ACCESS                             |
 * |__________________________________________________________________________|
*/
static bool pca96xx_regmap_is_writeable( struct device *dev, unsigned int reg ) {
	bool able = false;

	switch ( reg ) {
		case REG_INPORT0:
		case REG_INPORT1:
			able = false;
			break;
		case REG_OUTPORT0:
		case REG_OUTPORT1:
		case REG_POL_INV_P0:
		case REG_POL_INV_P1:
		case REG_CONF_PORT0:
		case REG_CONF_PORT1:
			able = true;
			break;
		default:
			able = false;
	}

	return able;
}


static bool pca96xx_regmap_is_volatile( struct device *dev, unsigned int reg ) {
		bool vol = false;

	switch ( reg ) {
		case REG_INPORT0:
		case REG_INPORT1:
			vol = true;
			break;
		case REG_OUTPORT0:
		case REG_OUTPORT1:
		case REG_POL_INV_P0:
		case REG_POL_INV_P1:
		case REG_CONF_PORT0:
		case REG_CONF_PORT1:
			vol = false;
			break;
		default:
			vol = false;
	}

	return vol;
}


static const struct regmap_config pca96xx_regmap_config = {
	.reg_bits      = 8,
	.val_bits      = 8,
	.cache_type    = REGCACHE_RBTREE,
	.max_register  = 0xFE,
	.writeable_reg = pca96xx_regmap_is_writeable,
	.volatile_reg  = pca96xx_regmap_is_volatile,
};
/* __________________________________________________________________________
* |__________________________________________________________________________|
*/


/*  __________________________________________________________________________
 * |                                                                          |
 * |                              W/R BASIC FUNCTION                          |
 * |__________________________________________________________________________|
*/
static inline uint8_t pca_read( pca96xx_t *drvdata, unsigned int reg ) {
	int      ret;
	uint32_t val;

	ret = regmap_read( drvdata->regmap, reg, &val );

	if ( ret ) {
		dev_err ( &drvdata->client->dev,
					"unable to read from reg%01x\n", reg );
		return 0;
	}

	return (uint8_t)val;
}


static inline int pca_set_bits( pca96xx_t *drvdata,
		unsigned int reg, uint8_t mask, uint8_t val )
{
	int ret;
	ret = regmap_update_bits( drvdata->regmap, (uint8_t)reg, mask, val);

	if ( ret ) {
		dev_err ( &drvdata->client->dev,
					"unable to write to reg%01x\n", reg );
		return 0;
	}

	return ret;
}


static inline int pca_write( pca96xx_t *drvdata,
		unsigned int reg, uint8_t val )
{
	int ret;
 	ret = regmap_write( drvdata->regmap, (uint8_t)reg, val);

	if ( ret ) {
		dev_err ( &drvdata->client->dev,
					"unable to write to reg%01x\n", reg );
		return 0;
	}

	return ret;
}
/* __________________________________________________________________________
* |__________________________________________________________________________|
*/

/*   ___________________________________________________________________________
 * |                                                                           |
 * |                            ADVANCED FUNCTIONS                             |
 * |___________________________________________________________________________|
*/
static inline unsigned gpio2bank( unsigned offset ) {
	return ( offset / BANK_SIZE );
}

static inline unsigned get_bankoffset( unsigned offset ) {
	return ( offset % BANK_SIZE );
}
/* __________________________________________________________________________
* |__________________________________________________________________________|
*/

/* __________________________________________________________________________
* |                                                                          |
* |                             GPIO CHIP INTERFACE                          |
* |__________________________________________________________________________|
*/
static int pca96xx_get_value( struct gpio_chip *chip, unsigned offset ) {
	pca96xx_t           *pca_chip;
	uint8_t            reg_value;
	uint8_t            mask = 1u << get_bankoffset( offset );
	int                status = 0;

	pca_chip = container_of( chip, pca96xx_t, gpio_chip );
	if ( !pca_chip )
		return -1;

	mutex_lock( &pca_chip->lock );
	reg_value = pca_read( pca_chip, gpio2bank( offset ) == 0 ? REG_INPORT0 : REG_INPORT1 );
	mutex_unlock( &pca_chip->lock );

	status = (int)(reg_value & mask);
	return status;
}


static void pca96xx_set_value (struct gpio_chip *chip, unsigned offset, int value) {
	pca96xx_t           *pca_chip;
	uint8_t            mask = 1u << get_bankoffset( offset );
	int                ret;

	pca_chip = container_of( chip, pca96xx_t, gpio_chip );
	if ( !pca_chip )
		return;

	mutex_lock( &pca_chip->lock );
	ret = pca_set_bits( pca_chip, gpio2bank( offset ) == 0 ? REG_OUTPORT0 : REG_OUTPORT1,
				mask, (uint8_t)value  << get_bankoffset( offset ) );
	mutex_unlock( &pca_chip->lock );

	if ( ret )
		PCA96XX_ERR( "cannot set outport register" );
}


static int pca96xx_direction_input (struct gpio_chip *chip, unsigned offset) {
	pca96xx_t           *pca_chip;
	uint8_t            mask = 1u << get_bankoffset( offset );
	int                ret;

	pca_chip = container_of( chip, pca96xx_t, gpio_chip );
	if ( !pca_chip )
		return -EINVAL;

	mutex_lock( &pca_chip->lock );
	ret = pca_set_bits( pca_chip, gpio2bank( offset ) == 0 ? REG_CONF_PORT0 : REG_CONF_PORT1,
				mask, DIR_IN << get_bankoffset( offset ) );
	mutex_unlock( &pca_chip->lock );

	return ret;
}


static int pca96xx_direction_output (struct gpio_chip *chip, unsigned offset, int value) {
	pca96xx_t           *pca_chip;
	uint8_t            mask = 1u << get_bankoffset( offset );
	int                ret;

	pca_chip = container_of( chip, pca96xx_t, gpio_chip );
	if ( !pca_chip )
		return -EINVAL;

	mutex_lock( &pca_chip->lock );
	ret = pca_set_bits( pca_chip, gpio2bank( offset ) == 0 ? REG_CONF_PORT0 : REG_CONF_PORT1,
				mask, DIR_OUT << get_bankoffset( offset ) );
	mutex_unlock( &pca_chip->lock );

	return ret;
}


static void pca96xx_dbg_show (struct seq_file *s, struct gpio_chip *chip) {
	pca96xx_t          *pca_chip;
	uint8_t            mask, inreg, polreg, dirreg;
	int                i;

	pca_chip = container_of( chip, pca96xx_t, gpio_chip );
	if ( !pca_chip )
		return;

	mutex_lock( &pca_chip->lock );

	for ( i = 0 ; i < chip->ngpio ; i++ ) {
		const char  *label;

		label = gpiochip_is_requested( chip, i );
		if (!label)
			continue;

		mask = 1u << get_bankoffset( i );
		if ( gpio2bank( i ) == 0 ) {
			inreg = REG_INPORT0;
			polreg = REG_POL_INV_P0;
			dirreg = REG_CONF_PORT0;
		} else {
			inreg = REG_INPORT1;
			polreg = REG_POL_INV_P1;
			dirreg = REG_CONF_PORT1;
		}

		seq_printf( s, " gpio-%-3d (%-20s) %s %s %s",
			chip->base + i, label,
			( pca_read( pca_chip, dirreg) & mask ) ? "in " : "out",
			( pca_read( pca_chip, inreg) & mask) ? "hi" : "lo",
			( pca_read( pca_chip, polreg) & mask) ? "1" : "0" );
		seq_printf(s, "\n");
	}

	mutex_unlock( &pca_chip->lock );
}
/* __________________________________________________________________________
* |__________________________________________________________________________|
*/


/* __________________________________________________________________________
* |                                                                          |
* |                                IRQ MANAGEMENT                            |
* |__________________________________________________________________________|
*/
static irqreturn_t pca96xx_hanlder_irq (int irq, void *data) {
	unsigned int  child_irq, i, state;
	uint16_t       input;
	pca96xx_t     *pca_chip = (pca96xx_t *)data;

	input = pca_read( pca_chip, REG_INPORT0 );
	input |= pca_read( pca_chip, REG_INPORT1 ) << BANK_SIZE;

	for ( i = 0 ; i < pca_chip->gpio_chip.ngpio ; i++ ) {

		if ( pca_chip->irq_used[i] ) {
			state = (BIT( i ) & input) >> i;
			if ( ( state && pca_chip->irq_rise[i] ) ||
					( !state && pca_chip->irq_fall[i] ) ) {

				child_irq = irq_find_mapping( pca_chip->irq_domain, i );
				handle_nested_irq( child_irq );
			}
		}

	}

	return IRQ_HANDLED;
}


static void noop( struct irq_data *data ) { }


static void pca96xx_free_irq( pca96xx_t *pca_chip ) {
	unsigned int irq, i;

	free_irq ( pca_chip->irq, pca_chip );

	for ( i = 0; i < pca_chip->gpio_chip.ngpio ; i++ ) {
		irq = irq_find_mapping( pca_chip->irq_domain, i );
		if ( irq > 0 )
			irq_dispose_mapping( irq );
	}

	irq_domain_remove( pca_chip->irq_domain );
}


static int pca96xx_gpio_to_irq (struct gpio_chip *chip, unsigned offset) {
	pca96xx_t *pca_chip;

	pca_chip = container_of( chip, pca96xx_t, gpio_chip );

	return irq_find_mapping( pca_chip->irq_domain, offset );
}


static int pca96xx_irq_set_type (struct irq_data *data, unsigned int type) {
	int           status;
	pca96xx_t     *pca_chip;
	unsigned int  pos = data->hwirq;

	pca_chip = irq_data_get_irq_chip_data( data );

	if ( (type & IRQ_TYPE_EDGE_BOTH) == IRQ_TYPE_EDGE_BOTH ) {
		pca_chip->irq_rise[pos] |= BIT(pos);
		pca_chip->irq_fall[pos] |= BIT(pos);
		status = 0;
	} else if ( type & IRQ_TYPE_EDGE_RISING ) {
		pca_chip->irq_rise[pos] |= BIT(pos);
		pca_chip->irq_fall[pos] &= ~BIT(pos);
		status = 0;
	} else if ( type & IRQ_TYPE_EDGE_FALLING ) {
		pca_chip->irq_fall[pos] |= BIT(pos);
		pca_chip->irq_rise[pos] &= ~BIT(pos);
		status = 0;
	} else {
		status = -EINVAL;
	}

	return status;
}


static void pca96xx_irq_bus_lock (struct irq_data *data) {
	pca96xx_t *pca_chip;

	pca_chip = irq_data_get_irq_chip_data( data );
	mutex_lock( &pca_chip->lock_irq );
}


static void pca96xx_irq_bus_unlock( struct irq_data *data ) {
	pca96xx_t *pca_chip;

	pca_chip = irq_data_get_irq_chip_data( data );
	mutex_unlock( &pca_chip->lock_irq );
}


static int pca96xx_irq_startup( struct irq_data *data ) {
	pca96xx_t     *pca_chip;
	unsigned int  pos = data->hwirq;

	pca_chip = irq_data_get_irq_chip_data( data );

	if ( gpiochip_lock_as_irq( &pca_chip->gpio_chip, data->hwirq ) ) {
		PCA96XX_ERR( "unable to lock HW IRQ %lu for IRQ usage", data->hwirq );
	}

	pca_chip->irq_used[pos] = 1;
	return 0;
}


static void pca96xx_irq_shutdown (struct irq_data *data) {
	pca96xx_t     *pca_chip;
	unsigned int  pos = data->hwirq;

	pca_chip = irq_data_get_irq_chip_data( data );
	gpiochip_unlock_as_irq( &pca_chip->gpio_chip, data->hwirq );

	pca_chip->irq_used[pos] = 0;
}


static struct irq_chip pca96xx_irq_chip = {
	.name                 = "pca96xx",
	.irq_mask             = noop,
	.irq_unmask           = noop,
	.irq_set_type         = pca96xx_irq_set_type,
	.irq_bus_lock         = pca96xx_irq_bus_lock,
	.irq_bus_sync_unlock  = pca96xx_irq_bus_unlock,
	.irq_request_resources          = pca96xx_irq_startup,
	.irq_release_resources         = pca96xx_irq_shutdown,
};
/* __________________________________________________________________________
* |__________________________________________________________________________|
*/



/* __________________________________________________________________________
* |                                                                          |
* |                                DRIVER SETUP                              |
* |__________________________________________________________________________|
*/
static int pca96xx_irq_setup( pca96xx_t *pca_chip ) {
	int err, irq, i, done;
	struct i2c_client *client = pca_chip->client;
	struct gpio_chip *chip = &pca_chip->gpio_chip;

	pca_chip->irq_domain = irq_domain_add_linear( chip->of_node, chip->ngpio,
								&irq_domain_simple_ops, pca_chip );
	if ( !pca_chip->irq_domain ) {
		PCA96XX_ERR( "cannot assign irq domain" );
		err = -ENODEV;
		goto err_irq_domain_add;
	}

	err = devm_request_threaded_irq( &client->dev, pca_chip->irq, NULL, pca96xx_hanlder_irq,
										IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
										dev_name(&client->dev), pca_chip) ;
	if ( err != 0 ) {
		PCA96XX_ERR( "unable to request IRQ#%d: %d", pca_chip->irq, err );
		goto err_request_thr_irq;
	}

	chip->to_irq = pca96xx_gpio_to_irq;

	for ( i = 0 ; i < chip->ngpio ; i++ ) {
		pca_chip->irq_used[i] = 0;
		pca_chip->irq_fall[i] = 0;
		pca_chip->irq_rise[i] = 0;

		irq = irq_create_mapping( pca_chip->irq_domain, i );
		if ( irq < 0 ) {
			PCA96XX_ERR( "cannot map irq" );
			done = i;
			err = -EINVAL;
			goto err_map_irq;
		}
		irq_set_lockdep_class( irq, &gpio_lock_class );
		irq_set_chip_data( irq, pca_chip );
		irq_set_chip( irq, &pca96xx_irq_chip );
		irq_set_nested_thread( irq, true );
	}

	return 0;
err_map_irq:
	while ( --done ) {
		irq = irq_find_mapping( pca_chip->irq_domain, i );
		if ( irq > 0 )
			irq_dispose_mapping( irq );
	}
err_request_thr_irq:
	irq_domain_remove( pca_chip->irq_domain );
err_irq_domain_add:
	return err;
}


static int pca96xx_setup( pca96xx_t *pca_chip, struct device *dev, unsigned base ) {
	int err, status;

	/* init gpio chip structure */
	pca_chip->gpio_chip.get              = pca96xx_get_value;
	pca_chip->gpio_chip.set              = pca96xx_set_value;
	pca_chip->gpio_chip.direction_input  = pca96xx_direction_input;
	pca_chip->gpio_chip.direction_output = pca96xx_direction_output;
	pca_chip->gpio_chip.dbg_show         = pca96xx_dbg_show;

	pca_chip->gpio_chip.of_gpio_n_cells = 2;
	pca_chip->gpio_chip.of_node         = dev->of_node;

	pca_chip->gpio_chip.ngpio = NR_GPIO;
	pca_chip->gpio_chip.label = PCA96xx_LBL;

	pca_chip->gpio_chip.base = base;
	pca_chip->gpio_chip.can_sleep = CAN_SLEEP;
	pca_chip->gpio_chip.parent = dev;
	pca_chip->gpio_chip.owner = THIS_MODULE;

	mutex_init( &pca_chip->lock );
	mutex_init( &pca_chip->lock_irq );

	/*  set all gpio as input  */
	pca_write( pca_chip, REG_CONF_PORT0, 0xFF );
	pca_write( pca_chip, REG_CONF_PORT1, 0xFF );

	/* add this gpio bank */
	status = gpiochip_add( &pca_chip->gpio_chip );
	if ( status < 0 ) {
		PCA96XX_ERR( "cannot add gpio bank to the system" );
		err = status;
		goto err_gpio_add;
	}

	status = pca96xx_irq_setup( pca_chip );
	if ( status ) {
		PCA96XX_ERR( "cannot setup gpio irq" );
		err = status;
		goto err_irq_setup;
	}

	return 0;
err_irq_setup:
err_gpio_add:
	return err;
}
/* __________________________________________________________________________
* |__________________________________________________________________________|
*/


static int pca96xx_probe( struct i2c_client *client, const struct i2c_device_id *id ) {
	pca96xx_t                     *drvdata;
	struct device_node		     *np = client->dev.of_node;
	int                          err, ret;
	unsigned base = -1;

	drvdata = devm_kzalloc( &client->dev, sizeof(*drvdata), GFP_KERNEL );
	if ( drvdata == NULL ) {
		PCA96XX_ERR( "unable to allocate driver data" );
		err = -ENOMEM;
		goto err_drvdata_alloc;
	}

	/*  DATA INITIALIZATION  */
	i2c_set_clientdata( client, drvdata );
	drvdata->client = client;

	drvdata->irq = irq_of_parse_and_map( np, 0 );

	drvdata->regmap = devm_regmap_init_i2c( client, &pca96xx_regmap_config );
	if ( IS_ERR(drvdata->regmap) ) {
		PCA96XX_ERR( "failed to allocate register map" );
		err = PTR_ERR( drvdata->regmap );
		goto err_regmap_init;
	}

	ret = pca96xx_setup( drvdata, &client->dev, base );
	if ( ret != 0 ) {
		PCA96XX_ERR( "cannot driver setup" );
		err = ret;
		goto err_setup;
	}

	return 0;
err_setup:
	regmap_exit( drvdata->regmap );
err_regmap_init:
	kfree( drvdata );
err_drvdata_alloc:
	return err;
}


static int pca96xx_remove( struct i2c_client *client ) {
	pca96xx_t  *drvdata = (pca96xx_t *)i2c_get_clientdata( client );

	pca96xx_free_irq( drvdata );

	gpiochip_remove( &drvdata->gpio_chip );

	kfree ( drvdata );

	return 0;
}


static const struct of_device_id pca96xx_match[] = {
	{ .compatible = "nxp,pca9655" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, pca96xx_match);


static const struct i2c_device_id pca96xx_i2c_ids[] = {
		{ "pca9655", 1 },
		{ }
};
MODULE_DEVICE_TABLE(i2c, pca96xx_i2c_ids);


static struct i2c_driver pca96xx_driver = {
	.driver = {
		.name		    = "pca96xx",
		.owner          = THIS_MODULE,
		.of_match_table	= pca96xx_match,
	},
	.probe    = pca96xx_probe,
	.remove   = pca96xx_remove,
	.id_table = pca96xx_i2c_ids,
};
module_i2c_driver(pca96xx_driver);

MODULE_AUTHOR("Davide Cardillo, SECO srl");
MODULE_DESCRIPTION("NXP PCA96xx GPIO expander family");
MODULE_LICENSE("GPL");
