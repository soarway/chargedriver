
#ifndef __BA70X_HEADER__
#define __BA70X_HEADER__

/* Register 00h */
#define BA70X_REG_00      		0x00

//uu#define BA70X_ENHIZ_MASK		    0x80
//uu#define BA70X_ENHIZ_SHIFT		    7
#define BA70X_REG_3C      		0x3C
#define BA70X_HIZ_EN_MASK		    0x80
#define BA70X_HIZ_EN_SHIFT		    7

#define BA70X_HIZ_ENABLE          1
#define BA70X_HIZ_DISABLE         0

#define BA70X_REG_34      		0x34
//uu#define BA70X_ENILIM_MASK		    0x40
//uu#define BA70X_ENILIM_SHIFT		6
#define BA70X_ENILIM_MASK		    0x80
#define BA70X_ENILIM_SHIFT		7

//uu 1 0可能要调换?//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
#define BA70X_ENILIM_ENABLE       1
#define BA70X_ENILIM_DISABLE      0

#define BA70X_IINLIM_MASK		    0x3F
#define BA70X_IINLIM_SHIFT		0
#define BA70X_IINLIM_BASE         100
#define BA70X_IINLIM_LSB          50

/* Register 01h */
#define BA70X_REG_01		    	0x01


//uu#define BA70X_BHOT_MASK           0xC0
//uu#define BA70X_BHOT_SHIFT          6
#define BA70X_REG_56		    	0x56
#define BA70X_BHOT_MASK           0x0C
#define BA70X_BHOT_SHIFT          2

//uu#define BA70X_BCOLD_MASK          0x20
//uu#define BA70X_BCOLD_SHIFT         5
#define BA70X_BCOLD_MASK          0x01
#define BA70X_BCOLD_SHIFT         0
//uu 变更5bit VINDPMOS-4bit VINDPM>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>计算值可能不对
//uu#define BA70X_VINDPMOS_MASK       0x1F
//uu#define BA70X_VINDPMOS_SHIFT      0
//#define BA70X_REG_33		    	0x33
#define BA70X_VINDPMOS_MASK       0x0F
#define BA70X_VINDPMOS_SHIFT      0

//uu#define BA70X_VINDPMOS_BASE       0
//uu#define BA70X_VINDPMOS_LSB        100
#define BA70X_VINDPMOS_BASE       4000
#define BA70X_VINDPMOS_LSB        100

/* Register 0x02 */
#define BA70X_REG_02              0x02
//uu#define BA70X_CONV_START_MASK      0x80
//uu#define BA70X_CONV_START_SHIFT     7
#define BA70X_REG_0F              0x0F//uu+

#define BA70X_CONV_START_MASK      0x80//uu no change
#define BA70X_CONV_START_SHIFT     7//uu no change
#define BA70X_CONV_START           0

#define BA70X_CONV_RATE_MASK       0x40
#define BA70X_CONV_RATE_SHIFT      6
//uu#define BA70X_ADC_CONTINUE_ENABLE  1
//uu#define BA70X_ADC_CONTINUE_DISABLE 0
#define BA70X_ADC_CONTINUE_ENABLE  0//ADC_RATE define against the 6970:1s Continuous conversion (default)
#define BA70X_ADC_CONTINUE_DISABLE 1//ADC_RATE define against the 6970:One-shot

//uu#define BA70X_BOOST_FREQ_MASK      0x20
//uu#define BA70X_BOOST_FREQ_SHIFT     5
//uu#define BA70X_BOOST_FREQ_1500K     0
//uu#define BA70X_BOOST_FREQ_500K      0
#define BA70X_REG_40              0x40//uu+
#define BA70X_BOOST_FREQ_MASK      0x18
#define BA70X_BOOST_FREQ_SHIFT     3
//uu 当前未使用，新更改为4档>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
#define BA70X_BOOST_FREQ_2500K     3
//#define BA70X_BOOST_FREQ_500K      2
#define BA70X_BOOST_FREQ_1500K     1
#define BA70X_BOOST_FREQ_500K      0

//uu#define BA70X_ICOEN_MASK          0x10
//uu#define BA70X_ICOEN_SHIFT         4
//uu#define BA70X_ICO_ENABLE          1
//uu#define BA70X_ICO_DISABLE         0
#define BA70X_REG_35              0x35//uu+
#define BA70X_ICOEN_MASK          0x40
#define BA70X_ICOEN_SHIFT         6
#define BA70X_ICO_ENABLE          1
#define BA70X_ICO_DISABLE         0


//uu#define BA70X_HVDCPEN_MASK        0x08
//uu#define BA70X_HVDCPEN_SHIFT       3
//uu#define BA70X_HVDCP_ENABLE        1
//uu#define BA70X_HVDCP_DISABLE       0
#define BA70X_REG_90              0x90//uu+ not be used
#define BA70X_HVDCPEN_MASK        0x20
#define BA70X_HVDCPEN_SHIFT       5
#define BA70X_HVDCP_ENABLE        1
#define BA70X_HVDCP_DISABLE       0

//bit2:HV_TYPE//not be used?
#define BA70X_MAXCEN_MASK         0x04
#define BA70X_MAXCEN_SHIFT        2
#define BA70X_MAXC_ENABLE         1
#define BA70X_MAXC_DISABLE        0

//uu#define BA70X_FORCE_DPDM_MASK     0x02
//uu#define BA70X_FORCE_DPDM_SHIFT    1
//uu#define BA70X_FORCE_DPDM          1
//reg2-1->reg90-7?      reg90-0:FC???
#define BA70X_FORCE_DPDM_MASK     0x80
#define BA70X_FORCE_DPDM_SHIFT    7
#define BA70X_FORCE_DPDM          1

//uu#define BA70X_AUTO_DPDM_EN_MASK   0x01
//uu#define BA70X_AUTO_DPDM_EN_SHIFT  0
//uu#define BA70X_AUTO_DPDM_ENABLE    1
//uu#define BA70X_AUTO_DPDM_DISABLE   0
//reg2-0->reg90-6
#define BA70X_AUTO_DPDM_EN_MASK   0x40
#define BA70X_AUTO_DPDM_EN_SHIFT  6
#define BA70X_AUTO_DPDM_ENABLE    1
#define BA70X_AUTO_DPDM_DISABLE   0

/* Register 0x03 */
#define BA70X_REG_03              0x03
//#define BA70X_BAT_LOADEN_MASK     0x80
//#define BA70X_BAT_LOAEN_SHIFT     7
//#define BA70X_REG_40              0x40//
#define BA70X_BAT_LOADEN_MASK     0x01//not be used
#define BA70X_BAT_LOAEN_SHIFT     0

//uu#define BA70X_WDT_RESET_MASK      0x40
//uu#define BA70X_WDT_RESET_SHIFT     6
//uu#define BA70X_WDT_RESET           1
#define BA70X_REG_07              0x07//03-6->07-3
#define BA70X_WDT_RESET_MASK      0x08
#define BA70X_WDT_RESET_SHIFT     3
#define BA70X_WDT_RESET           1
//未找到对应bit>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>!!!!!
#define BA70X_OTG_CONFIG_MASK     0x20
#define BA70X_OTG_CONFIG_SHIFT    5
#define BA70X_OTG_ENABLE          1
#define BA70X_OTG_DISABLE         0

//uu#define BA70X_CHG_CONFIG_MASK     0x10
//uu#define BA70X_CHG_CONFIG_SHIFT    4
//uu#define BA70X_CHG_ENABLE          1
//uu#define BA70X_CHG_DISABLE         0
//#define BA70X_REG_3C              0x3C//03-4->3C-0
#define BA70X_CHG_CONFIG_MASK     0x01
#define BA70X_CHG_CONFIG_SHIFT    0
#define BA70X_CHG_ENABLE          1
#define BA70X_CHG_DISABLE         0

//uu#define BA70X_SYS_MINV_MASK       0x0E
//uu#define BA70X_SYS_MINV_SHIFT      1
//uu#define BA70X_SYS_MINV_BASE       3000
//uu#define BA70X_SYS_MINV_LSB        100
#define BA70X_REG_30              0x30//03-3:1->30-2:0
#define BA70X_SYS_MINV_MASK       0x07//not be used
#define BA70X_SYS_MINV_SHIFT      0
#define BA70X_SYS_MINV_BASE       2600
#define BA70X_SYS_MINV_LSB        100


/* Register 0x04*/
#define BA70X_REG_04              0x04
//uu#define BA70X_EN_PUMPX_MASK       0x80
//uu#define BA70X_EN_PUMPX_SHIFT      7
//uu#define BA70X_PUMPX_ENABLE        1
//uu#define BA70X_PUMPX_DISABLE       0
#define BA70X_REG_57              0x57//04-7->57-4
#define BA70X_EN_PUMPX_MASK       0x10
#define BA70X_EN_PUMPX_SHIFT      4
#define BA70X_PUMPX_ENABLE        1
#define BA70X_PUMPX_DISABLE       0

//uu#define BA70X_ICHG_MASK           0x7F
//uu#define BA70X_ICHG_SHIFT          0
//uu#define BA70X_ICHG_BASE           0
//uu#define BA70X_ICHG_LSB            64
#define BA70X_REG_32              0x32//04-6:0->32-6:0
#define BA70X_ICHG_MASK           0x7F
#define BA70X_ICHG_SHIFT          0
#define BA70X_ICHG_BASE           0
#define BA70X_ICHG_LSB            50 //0mA (0000000) – 4000mA (1010000~1111111)

/* Register 0x05*/
#define BA70X_REG_05              0x05
//uu#define BA70X_IPRECHG_MASK        0xF0
//uu#define BA70X_IPRECHG_SHIFT       4
//uu#define BA70X_IPRECHG_BASE        64
//uu#define BA70X_IPRECHG_LSB         64
#define BA70X_REG_36              0x36//05-7:4->36-3:0
#define BA70X_IPRECHG_MASK        0x0F
#define BA70X_IPRECHG_SHIFT       0
#define BA70X_IPRECHG_BASE        50
#define BA70X_IPRECHG_LSB         50

//uu#define BA70X_ITERM_MASK          0x0F
//uu#define BA70X_ITERM_SHIFT         0
//uu#define BA70X_ITERM_BASE          64
//uu#define BA70X_ITERM_LSB           64
#define BA70X_REG_37              0x37//05-3:0->37-4:0//增加1bit>>>>>>>>>>>>>>>>>
#define BA70X_ITERM_MASK          0x1F
#define BA70X_ITERM_SHIFT         0
#define BA70X_ITERM_BASE          100
#define BA70X_ITERM_LSB           50//增加两档37-6 select 50/25
#define BA70X_ITERM_LSB1          25//增加两档//uu add>>>>>>>>>>>>>>>>>>>>>>>!!!
/* Register 0x06*/
#define BA70X_REG_06              0x06
//uu#define BA70X_VREG_MASK           0xFC
//uu#define BA70X_VREG_SHIFT          2
//uu#define BA70X_VREG_BASE           3840
//uu#define BA70X_VREG_LSB            16
#define BA70X_REG_31              0x31//06-5:0->31-6:0
#define BA70X_VREG_MASK           0x7F////增加1bit>>>>>>>>>>>>>>>>>
#define BA70X_VREG_SHIFT          0
#define BA70X_VREG_BASE           3840
#define BA70X_VREG_LSB            8


//uu#define BA70X_BATLOWV_MASK        0x02
//uu#define BA70X_BATLOWV_SHIFT       1
//uu#define BA70X_BATLOWV_2800MV      0
//uu#define BA70X_BATLOWV_3000MV      1
//#define BA70X_REG_36              0x36//06-1->36-7:6
#define BA70X_BATLOWV_MASK        0xC0//增加1bit>>>>>>>>>>>>>>>>>
#define BA70X_BATLOWV_SHIFT       6//not be used>>>>>>>>
#define BA70X_BATLOWV_2900MV      0
#define BA70X_BATLOWV_3000MV      1
#define BA70X_BATLOWV_3100MV      2
#define BA70X_BATLOWV_3200MV      3

//uu#define BA70X_VRECHG_MASK         0x01
//uu#define BA70X_VRECHG_SHIFT        0
//uu#define BA70X_VRECHG_100MV        0
//uu#define BA70X_VRECHG_200MV        1
#define BA70X_REG_38              0x38//06-0->38-1:0
#define BA70X_VRECHG_MASK         0x03//增加1bit>>>>>>>>>>>>>>>>>
#define BA70X_VRECHG_SHIFT        0//not be used>>>>>>>>
#define BA70X_VRECHG_100MV        0
#define BA70X_VRECHG_200MV        1
#define BA70X_VRECHG_300MV        2
#define BA70X_VRECHG_400MV        3
/* Register 0x07*/
#define BA70X_REG_07              0x07
//uu#define BA70X_EN_TERM_MASK        0x80
//uu#define BA70X_EN_TERM_SHIFT       7
//uu#define BA70X_TERM_ENABLE         1
//uu#define BA70X_TERM_DISABLE        0
//#define BA70X_REG_37              0x37
#define BA70X_EN_TERM_MASK        0x80
#define BA70X_EN_TERM_SHIFT       7
#define BA70X_TERM_ENABLE         1
#define BA70X_TERM_DISABLE        0

#define BA70X_STAT_DIS 0//uu+ not be used 07-6

//uu#define BA70X_WDT_MASK            0x30
//uu#define BA70X_WDT_SHIFT           4
//uu#define BA70X_WDT_DISABLE         0
//uu#define BA70X_WDT_40S             1
//uu#define BA70X_WDT_80S             2
//uu#define BA70X_WDT_160S            3
//uu#define BA70X_WDT_BASE            0
//uu#define BA70X_WDT_LSB             40
//#define BA70X_REG_07           0x07//07-5:4->07-2:0//增加1bit>>>>>>>>>>>>>>>>>
#define BA70X_WDT_MASK            0x07
#define BA70X_WDT_SHIFT           0
#define BA70X_WDT_DISABLE         0
#define BA70X_WDT_0S5             1
#define BA70X_WDT_1S              2
#define BA70X_WDT_2S              3
#define BA70X_WDT_20S             4
#define BA70X_WDT_40S             5
#define BA70X_WDT_80S             6
#define BA70X_WDT_160S            7
#define BA70X_WDT_BASE            0
#define BA70X_WDT_LSB             40


//uu#define BA70X_EN_TIMER_MASK       0x08
//uu#define BA70X_EN_TIMER_SHIFT      3
#define BA70X_REG_3F              0x3F//07-3->3F-6
#define BA70X_EN_TIMER_MASK       0x40//not be used
#define BA70X_EN_TIMER_SHIFT      6
#define BA70X_CHG_TIMER_ENABLE    1//not be used
#define BA70X_CHG_TIMER_DISABLE   0

//uu#define BA70X_CHG_TIMER_MASK      0x06//07-1:0->3F-5:4
//uu#define BA70X_CHG_TIMER_SHIFT     1
//uu#define BA70X_CHG_TIMER_5HOURS    0
//uu#define BA70X_CHG_TIMER_8HOURS    1
//uu#define BA70X_CHG_TIMER_12HOURS   2
//uu#define BA70X_CHG_TIMER_20HOURS   3
#define BA70X_CHG_TIMER_MASK      0x30//07-1:0->3F-5:4
#define BA70X_CHG_TIMER_SHIFT     4
#define BA70X_CHG_TIMER_5HOURS    0
#define BA70X_CHG_TIMER_8HOURS    1
#define BA70X_CHG_TIMER_12HOURS   2
#define BA70X_CHG_TIMER_24HOURS   3

//uu#define BA70X_JEITA_ISET_MASK     0x01//07-0->3E-1
//uu#define BA70X_JEITA_ISET_SHIFT    0
//uu#define BA70X_JEITA_ISET_50PCT    0
//uu#define BA70X_JEITA_ISET_20PCT    1
#define BA70X_JEITA_ISET_MASK     0x02//07-0->3E-1
#define BA70X_JEITA_ISET_SHIFT    1//not be used
#define BA70X_JEITA_ISET_50PCT    0
#define BA70X_JEITA_ISET_20PCT    1

/* Register 0x08*/
#define BA70X_REG_08              0x08
//uu#define BA70X_BAT_COMP_MASK       0xE0
//uu#define BA70X_BAT_COMP_SHIFT      5
//uu#define BA70X_BAT_COMP_BASE       0
//uu#define BA70X_BAT_COMP_LSB        20
#define BA70X_REG_3E              0x3E//not be used
#define BA70X_BAT_COMP_MASK       0xE0
#define BA70X_BAT_COMP_SHIFT      5
#define BA70X_BAT_COMP_BASE       0
#define BA70X_BAT_COMP_LSB        10

//uu#define BA70X_VCLAMP_MASK         0x1C
//uu#define BA70X_VCLAMP_SHIFT        2
//uu#define BA70X_VCLAMP_BASE         0
//uu#define BA70X_VCLAMP_LSB          32
#define BA70X_VCLAMP_MASK         0x1C
#define BA70X_VCLAMP_SHIFT        2
#define BA70X_VCLAMP_BASE         0
#define BA70X_VCLAMP_LSB          16

//uu#define BA70X_TREG_MASK           0x03
//uu#define BA70X_TREG_SHIFT          0
//uu#define BA70X_TREG_60C            0
//uu#define BA70X_TREG_80C            1
//uu#define BA70X_TREG_100C           2
//uu#define BA70X_TREG_120C           3
//#define BA70X_REG_3F              0x3F//not be used
#define BA70X_TREG_MASK           0x06
#define BA70X_TREG_SHIFT          1
#define BA70X_TREG_60C            0
#define BA70X_TREG_80C            1
#define BA70X_TREG_100C           2
#define BA70X_TREG_120C           3


/* Register 0x09*/
#define BA70X_REG_09              0x09
//uu#define BA70X_FORCE_ICO_MASK      0x80
//uu#define BA70X_FORCE_ICO_SHIFT     7
//uu#define BA70X_FORCE_ICO           1
#define BA70X_REG_35              0x35
#define BA70X_FORCE_ICO_MASK      0x80//reg09-7->reg35-7
#define BA70X_FORCE_ICO_SHIFT     7
#define BA70X_FORCE_ICO           1

//uu#define BA70X_TMR2X_EN_MASK       0x40
//uu#define BA70X_TMR2X_EN_SHIFT      6
//#define BA70X_REG_3F              0x3F//09-6->3F-7
#define BA70X_TMR2X_EN_MASK       0x80//not be used
#define BA70X_TMR2X_EN_SHIFT      7

//uu#define BA70X_BATFET_DIS_MASK     0x20
//uu#define BA70X_BATFET_DIS_SHIFT    5
//uu#define BA70X_BATFET_OFF          1
#define BA70X_REG_3B              0x3B//09-5->3B-1
#define BA70X_BATFET_DIS_MASK     0x02
#define BA70X_BATFET_DIS_SHIFT    1
#define BA70X_BATFET_OFF          1

//uu#define BA70X_JEITA_VSET_MASK     0x10
//uu#define BA70X_JEITA_VSET_SHIFT    4
//uu#define BA70X_JEITA_VSET_N150MV   0
//uu#define BA70X_JEITA_VSET_VREG     1
//#define BA70X_REG_3E              0x3E//09-4->3E-0
#define BA70X_JEITA_VSET_MASK     0x01//not be used
#define BA70X_JEITA_VSET_SHIFT    0
#define BA70X_JEITA_VSET_N200MV   0
#define BA70X_JEITA_VSET_VREG     1

//#define BA70X_REG_3B              0x3B//09-3->3B-2
#define BA70X_BATFET_DLY_MASK     0x04
#define BA70X_BATFET_DLY_SHIFT    0x02//uu+

//uu#define BA70X_BATFET_RST_EN_MASK  0x04
//uu#define BA70X_BATFET_RST_EN_SHIFT 2
//#define BA70X_REG_3B              0x3B//09-2->3B-3
#define BA70X_BATFET_RST_EN_MASK  0x08
#define BA70X_BATFET_RST_EN_SHIFT 3

//uu#define BA70X_PUMPX_UP_MASK       0x02
//uu#define BA70X_PUMPX_UP_SHIFT      1
//uu#define BA70X_PUMPX_UP            1
//uu#define BA70X_PUMPX_DOWN_MASK     0x01
//uu#define BA70X_PUMPX_DOWN_SHIFT    0
//uu#define BA70X_PUMPX_DOWN          1
#define BA70X_REG_58              0x58//09-1->58-5;09-0->58-4
#define BA70X_PUMPX_UP_MASK       0x20
#define BA70X_PUMPX_UP_SHIFT      5
#define BA70X_PUMPX_UP            1
#define BA70X_PUMPX_DOWN_MASK     0x10
#define BA70X_PUMPX_DOWN_SHIFT    4
#define BA70X_PUMPX_DOWN          1
/* Register 0x0A*/
#define BA70X_REG_0A              0x0A
//uu#define BA70X_BOOSTV_MASK         0xF0
//uu#define BA70X_BOOSTV_SHIFT        4
//uu#define BA70X_BOOSTV_BASE         4550
//uu#define BA70X_BOOSTV_LSB          64
#define BA70X_REG_39              0x39//0A-7:4->39-7:3
//UU#define BA70X_REG_0A              0x0A//增加1bit>>>>>>>>>>>>>>>>>
#define BA70X_BOOSTV_MASK         0xF8
#define BA70X_BOOSTV_SHIFT        3
#define BA70X_BOOSTV_BASE         3900
#define BA70X_BOOSTV_LSB          100

//uu#define BA70X_BOOST_LIM_MASK      0x07
//uu#define BA70X_BOOST_LIM_SHIFT     0
//uu#define BA70X_BOOST_LIM_500MA     0x00
//uu#define BA70X_BOOST_LIM_700MA     0x01
//uu#define BA70X_BOOST_LIM_1100MA    0x02
//uu#define BA70X_BOOST_LIM_1300MA    0x03
//uu#define BA70X_BOOST_LIM_1600MA    0x04
//uu#define BA70X_BOOST_LIM_1800MA    0x05
//uu#define BA70X_BOOST_LIM_2100MA    0x06
//uu#define BA70X_BOOST_LIM_2400MA    0x07
//#define BA70X_REG_39              0x39//0A-2:0->39-2:0
#define BA70X_BOOST_LIM_MASK      0x07
#define BA70X_BOOST_LIM_SHIFT     0
#define BA70X_BOOST_LIM_500MA     0x00
#define BA70X_BOOST_LIM_900MA     0x01
#define BA70X_BOOST_LIM_1300MA    0x02
#define BA70X_BOOST_LIM_1500MA    0x03
#define BA70X_BOOST_LIM_2100MA    0x04
#define BA70X_BOOST_LIM_2500MA    0x05
//uu#define BA70X_BOOST_LIM_2100MA    0x06
//uu#define BA70X_BOOST_LIM_2400MA    0x07

/* Register 0x0B*/
#define BA70X_REG_0B              0x0B
//uu#define BA70X_VBUS_STAT_MASK      0xE0           
//uu#define BA70X_VBUS_STAT_SHIFT     5
#define BA70X_REG_94              0x94//0B-7:5->94-7:5
#define BA70X_VBUS_STAT_MASK      0xE0           
#define BA70X_VBUS_STAT_SHIFT     5

//uu#define BA70X_CHRG_STAT_MASK      0x18
//uu#define BA70X_CHRG_STAT_SHIFT     3
//uu#define BA70X_CHRG_STAT_IDLE      0
//uu#define BA70X_CHRG_STAT_PRECHG    1
//uu#define BA70X_CHRG_STAT_FASTCHG   2
//uu#define BA70X_CHRG_STAT_CHGDONE   3
#define BA70X_REG_42              0x42//0B-4:3->42-7:5
#define BA70X_CHRG_STAT_MASK      0xE0//增加1bit>>>>>>>>>>>>>>>>>
#define BA70X_CHRG_STAT_SHIFT     5
#define BA70X_CHRG_STAT_IDLE      0
#define BA70X_CHRG_STAT_TRICKLE    1//uu+
#define BA70X_CHRG_STAT_PRECHG     2
#define BA70X_CHRG_STAT_FASTCHG_CC 3//uu+
#define BA70X_CHRG_STAT_FASTCHG_CV 4//uu
#define BA70X_CHRG_STAT_CHGDONE    5

//uu#define BA70X_PG_STAT_MASK        0x04
//uu#define BA70X_PG_STAT_SHIFT       2
#define BA70X_REG_41              0x41//0B-2->41-0
#define BA70X_PG_STAT_MASK        0x01
#define BA70X_PG_STAT_SHIFT       0

#define BA70X_SDP_STAT_MASK       0x02//uu:SDP_STAT not be find and used
#define BA70X_SDP_STAT_SHIFT      1//uu:SDP_STAT not be find in ba76

//uu#define BA70X_VSYS_STAT_MASK      0x01
//uu#define BA70X_VSYS_STAT_SHIFT     0
//#define BA70X_REG_42              0x42//0B-0->42-3
#define BA70X_VSYS_STAT_MASK      0x08//not be used
#define BA70X_VSYS_STAT_SHIFT     3


/* Register 0x0C*/
#define BA70X_REG_0C              0x0c//uu: not be find in ba76
#define BA70X_FAULT_WDT_MASK      0x80
#define BA70X_FAULT_WDT_SHIFT     7
#define BA70X_FAULT_BOOST_MASK    0x40
#define BA70X_FAULT_BOOST_SHIFT   6
#define BA70X_FAULT_CHRG_MASK     0x30
#define BA70X_FAULT_CHRG_SHIFT    4
#define BA70X_FAULT_CHRG_NORMAL   0
#define BA70X_FAULT_CHRG_INPUT    1
#define BA70X_FAULT_CHRG_THERMAL  2
#define BA70X_FAULT_CHRG_TIMER    3

#define BA70X_FAULT_BAT_MASK      0x08
#define BA70X_FAULT_BAT_SHIFT     3
#define BA70X_FAULT_NTC_MASK      0x07
#define BA70X_FAULT_NTC_SHIFT     0
#define BA70X_FAULT_NTC_TSCOLD    1
#define BA70X_FAULT_NTC_TSHOT     2

#define BA70X_FAULT_NTC_WARM      2
#define BA70X_FAULT_NTC_COOL      3
#define BA70X_FAULT_NTC_COLD      5
#define BA70X_FAULT_NTC_HOT       6


/* Register 0x0D*/
#define BA70X_REG_0D              0x0D
//uu#define BA70X_FORCE_VINDPM_MASK   0x80        
//uu#define BA70X_FORCE_VINDPM_SHIFT  7
//uu#define BA70X_FORCE_VINDPM_ENABLE 1
//uu#define BA70X_FORCE_VINDPM_DISABLE 0
#define BA70X_REG_33              0x33 //0D-7->33-4
#define BA70X_FORCE_VINDPM_MASK   0x10        
#define BA70X_FORCE_VINDPM_SHIFT  4
#define BA70X_FORCE_VINDPM_ENABLE 1//DIS-DIS
#define BA70X_FORCE_VINDPM_DISABLE 0

//uu#define BA70X_VINDPM_MASK         0x7F
//uu#define BA70X_VINDPM_SHIFT        0
//uu#define BA70X_VINDPM_BASE         2600
//uu#define BA70X_VINDPM_LSB          100
//#define BA70X_REG_33              0x33 //0D-6:0->33-3:0
#define BA70X_VINDPM_MASK         0x0F//减少3bits
#define BA70X_VINDPM_SHIFT        0
#define BA70X_VINDPM_BASE         4000//may be error>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
#define BA70X_VINDPM_LSB          100
//Range: 4V (0000) – 4.8V (1000)~100mV/step, 7.6V((1001), 8.2V(1010), 8.4V(1011), 8.6V(1100), 10V(1101), 10.5V(1110), 10.7V(1111)Default: 4.5V (0101)


/* Register 0x0E*/
#define BA70X_REG_0E              0x0E
//uu#define BA70X_THERM_STAT_MASK     0x80
//uu#define BA70X_THERM_STAT_SHIFT    7
#define BA70X_REG_43              0x43//0E-7->43-6
#define BA70X_THERM_STAT_MASK     0x40
#define BA70X_THERM_STAT_SHIFT    6

//uu#define BA70X_BATV_MASK           0x7F
//uu#define BA70X_BATV_SHIFT          0
//uu#define BA70X_BATV_BASE           2304
//uu#define BA70X_BATV_LSB            20
//#define BA70X_REG_31              0x31//0E-6:0->31-6:0
#define BA70X_BATV_MASK           0x7F//may be error>>>>>>>>>>>>>>>>>>>>>>>>>
#define BA70X_BATV_SHIFT          0
#define BA70X_BATV_BASE           3840//may be error>>>>>>>>>>>>>>>>>>>>>>>>>
#define BA70X_BATV_LSB            8
//Charge Voltage Limit Fixed offset: 3.84V Bit step size: 8mV 
//Range: 3.84V-4.856V Default: 4.208V (0101110)

/* Register 0x0F*/
#define BA70X_REG_0F              0x0F
#define BA70X_SYSV_MASK           0x7F
#define BA70X_SYSV_SHIFT          0
#define BA70X_SYSV_BASE           2304
#define BA70X_SYSV_LSB            20


/* Register 0x10*/
#define BA70X_REG_10              0x10
#define BA70X_TSPCT_MASK          0x7F
#define BA70X_TSPCT_SHIFT         0
#define BA70X_TSPCT_BASE          21
#define BA70X_TSPCT_LSB           465//should be 0.465,kernel does not support float

/* Register 0x11*/
#define BA70X_REG_11              0x11
#define BA70X_VBUS_GD_MASK        0x80
#define BA70X_VBUS_GD_SHIFT       7
#define BA70X_VBUSV_MASK          0x7F
#define BA70X_VBUSV_SHIFT         0
#define BA70X_VBUSV_BASE          2600
#define BA70X_VBUSV_LSB           100


/* Register 0x12*/
#define BA70X_REG_12              0x12
#define BA70X_ICHGR_MASK          0x7F
#define BA70X_ICHGR_SHIFT         0
#define BA70X_ICHGR_BASE          0
#define BA70X_ICHGR_LSB           50


/* Register 0x13*/
#define BA70X_REG_13              0x13
#define BA70X_VDPM_STAT_MASK      0x80
#define BA70X_VDPM_STAT_SHIFT     7
#define BA70X_IDPM_STAT_MASK      0x40
#define BA70X_IDPM_STAT_SHIFT     6
#define BA70X_IDPM_LIM_MASK       0x3F
#define BA70X_IDPM_LIM_SHIFT      0
#define BA70X_IDPM_LIM_BASE       100
#define BA70X_IDPM_LIM_LSB        50


/* Register 0x14*/
#define BA70X_REG_14              0x14
#define BA70X_RESET_MASK          0x80             
#define BA70X_RESET_SHIFT         7
#define BA70X_RESET               1
#define BA70X_ICO_OPTIMIZED_MASK  0x40
#define BA70X_ICO_OPTIMIZED_SHIFT 6
#define BA70X_PN_MASK             0x38
#define BA70X_PN_SHIFT            3
#define BA70X_TS_PROFILE_MASK     0x04
#define BA70X_TS_PROFILE_SHIFT    2
#define BA70X_DEV_REV_MASK        0x03
#define BA70X_DEV_REV_SHIFT       0



/*
/////////////////////////////////////////
#define Reserved                  0
//00
#define BA70X_REG_00              0x00
#define BA70X_DEVICE_ID_MASK      0xFF
#define BA70X_DEVICE_ID_SHIFT     0
#define BA70X_DEVICE_ID_BASE      0x66

//01
#define BA70X_REG_01              0x01
//02
#define BA70X_REG_02              0x02
//03
#define BA70X_REG_03              0x03

//04
#define BA70X_REG_04              0x04
#define BA70X_VAC_OVP_DIS_MASK    0x80
#define BA70X_VAC_OVP_DIS_SHIFT   7
#define BA70X_VAC_OVP_DIS_ENABLE  0
#define BA70X_VAC_OVP_DIS_DISABLE 1

#define BA70X_VAC_OVP_MASK        0x70
#define BA70X_VAC_OVP_SHIFT       4
#define BA70X_VAC_OVP_BASE0       6500//?
#define BA70X_VAC_OVP_BASE1       11000
#define BA70X_VAC_OVP_LSB         1000

#define BA70X_REG_04_3            Reserved

#define BA70X_VBUS_OVP_DIS_MASK    0x04
#define BA70X_VBUS_OVP_DIS_SHIFT   2
#define BA70X_VBUS_OVP_DIS_ENABLE  0
#define BA70X_VBUS_OVP_DIS_DISABLE 1

#define BA70X_VBUS_OVP_MASK        0x03
#define BA70X_VBUS_OVP_SHIFT       0
#define BA70X_VBUS_OVP_BASE0       6000
#define BA70X_VBUS_OVP_BASE1       10000
#define BA70X_VBUS_OVP_LSB         2000
//05//TSBUS_FLT=0%+[TSBUS_FLT]*0.19531%//Range:0% - 50%//00010101 – 4.1% (default)
#define BA70X_REG_05               0x05
#define BA70X_TSBUS_FLT_MASK       0xFF
#define BA70X_TSBUS_FLT_SHIFT      0
#define BA70X_TSBUS_FLT_BASE       0
#define BA70X_TSBUS_FLT_LSB        19531
//06//TSBAT_FLT=0%+[TSBUS_FLT]*0.19531%//Range:0% - 50%//00010101 – 4.1% (default)
#define BA70X_REG_06               0x06
#define BA70X_TSBAT_FLT_MASK       0xFF
#define BA70X_TSBAT_FLT_SHIFT      0
#define BA70X_TSBAT_FLT_BASE       0
#define BA70X_TSBAT_FLT_LSB        19531
//07
#define BA70X_REG_07               0x07
#define BA70X_REG_07_7             Reserved
#define BA70X_REG_07_6             Reserved

#define BA70X_ACDRV_EN_MASK        0x20
#define BA70X_ACDRV_EN_SHIFT       5
#define BA70X_ACDRV_EN_ENABLE      1
#define BA70X_ACDRV_EN_DISABLE     0

#define BA70X_ACDRV_MANUAL_EN_MASK       0x10
#define BA70X_ACDRV_MANUAL_EN_SHIFT      4
#define BA70X_ACDRV_MANUAL_EN_ENABLE     1//Manual-mode
#define BA70X_ACDRV_MANUAL_EN_DISABLE    0//Auto-mode (default)

#define BA70X_WD_TIMER_RST_MASK        0x08
#define BA70X_WD_TIMER_RST_SHIFT       3
#define BA70X_WD_TIMER_RST_ENABLE      1//Back to 0 after timer reset
#define BA70X_WD_TIMER_RST_DISABLE     0//Normal (default)

#define BA70X_WD_TIMEOUT_MASK        0x07
#define BA70X_WD_TIMEOUT_SHIFT       0
#define BA70X_WD_TIMEOUT_DISABLE     0
#define BA70X_WD_TIMEOUT_0S5         1
#define BA70X_WD_TIMEOUT_1S          2
#define BA70X_WD_TIMEOUT_2S          3 
#define BA70X_WD_TIMEOUT_20S         4
#define BA70X_WD_TIMEOUT_40S         5
#define BA70X_WD_TIMEOUT_80S         6
#define BA70X_WD_TIMEOUT_160S        7
#define BA70X_WD_TIMEOUT_BASE        0//?
#define BA70X_WD_TIMEOUT_LSB         1//?
//#define BA70X_WDT_MASK            0x30
//#define BA70X_WDT_SHIFT           4
//#define BA70X_WDT_DISABLE         0
//#define BA70X_WDT_40S             1
//#define BA70X_WDT_80S             2
//#define BA70X_WDT_160S            3
//#define BA70X_WDT_BASE            0
//#define BA70X_WDT_LSB             40

//08
#define BA70X_REG_08               0x08
#define BA70X_REG_RST_MASK         0x80
#define BA70X_REG_RST_SHIFT        7
#define BA70X_REG_RST              1//Reset to default register value and reset safety timer
#define BA70X_REG_RST_N            0//Keep current register setting (default)
//#define BA70X_REG_14              0x14
//#define BA70X_RESET_MASK          0x80             
//#define BA70X_RESET_SHIFT         7
//#define BA70X_RESET               1


//PD(pulls down):When this bit is enabled, it pulls down the VBUS with 100mA (IBUS_PD)
#define BA70X_VBUS_PD_MASK       0x40
#define BA70X_VBUS_PD_SHIFT      6
#define BA70X_VBUS_PD_ENABLE     1
#define BA70X_VBUS_PD_DISABLE    0

//PD(pulls down):When this bit is enabled, it pulls down the VAC with 50mA (IVAC_PD)
#define BA70X_VAC_PD_MASK       0x20
#define BA70X_VAC_PD_SHIFT      5
#define BA70X_VAC_PD_ENABLE     1
#define BA70X_VAC_PD_DISABLE    0

//PD(pulls down):When this bit is enabled, it pulls down the PMID with 30mA (IPMID_PD)
#define BA70X_PMID_PD_EN_MASK       0x10
#define BA70X_PMID_PD_EN_SHIFT      4
#define BA70X_PMID_PD_EN_ENABLE     1
#define BA70X_PMID_PD_EN_DISABLE    0

#define BA70X_REG_08_3             Reserved
#define BA70X_REG_08_2             Reserved

#define BA70X_TSHUT_DIS_MASK       0x02
#define BA70X_TSHUT_DIS_SHIFT      1
#define BA70X_TSHUT_DIS_N          0//Enable protection (default)//Use _N for easy to understand
#define BA70X_TSHUT_DIS            1//Disable protection

#define BA70X_TSBUS_TSBAT_FLT_DIS_MASK       0x01
#define BA70X_TSBUS_TSBAT_FLT_DIS_SHIFT      0
#define BA70X_TSBUS_TSBAT_FLT_DIS_N          0//Enable the TSBUS and TSBAT fault protection (default)
#define BA70X_TSBUS_TSBAT_FLT_DIS            1//Disable the TSBUS and TSBAT fault protection

//09
#define BA70X_REG_09_7             Reserved
#define BA70X_REG_09_6             Reserved
#define BA70X_REG_09_5             Reserved

#define BA70X_ADC_DONE_STAT_MASK       0x10
#define BA70X_ADC_DONE_STAT_SHIFT      4
//Old version has not define as the follow.
#define BA70X_ADC_DONE_STAT_N     0//Indicates the ADC conversion have not completed(default)
#define BA70X_ADC_DONE_STAT       1//Indicates the ADC conversion is complete in 1-shot mode only

#define BA70X_REGN_OK_STAT_MASK        0x08
#define BA70X_REGN_OK_STAT_SHIFT       3
#define BA70X_REGN_OK_STAT_N           0//The REGN voltage is lower than VREGN_OK falling threshold (default)
#define BA70X_REGN_OK_STAT             1//The REGN voltage is higher than VREGN_OK rising threshold

#define BA70X_VBAT_PRESENT_STAT_MASK   0x04
#define BA70X_VBAT_PRESENT_STAT_SHIFT  2
#define BA70X_VBAT_PRESENT_STAT_N      0//The VBAT voltage is lower than VBAT_UVLO Falling threshold (default)
#define BA70X_VBAT_PRESENT_STAT        1//The VBAT voltage is higher than VBAT_UVLO rising threshold

#define BA70X_VBUS_PRESENT_STAT_MASK   0x02
#define BA70X_VBUS_PRESENT_STAT_SHIFT  1
#define BA70X_VBUS_PRESENT_STAT_N      0//The VBUS voltage is lower than VBUS_PRESENT falling threshold (default)
#define BA70X_VBUS_PRESENT_STAT        1//The VBUS voltage is higher than VBUS_PRESENT rising threshold

#define BA70X_VAC_PRESENT_STAT_MASK   0x01
#define BA70X_VAC_PRESENT_STAT_SHIFT  0
#define BA70X_VAC_PRESENT_STAT_N      0//The VAC voltage is lower than VAC_PRESENT falling threshold (default)
#define BA70X_VAC_PRESENT_STAT        1//The VAC voltage is higher than VAC_PRESENT rising threshold

//0A
#define BA70X_REG_0A_7                Reserved

#define BA70X_POR_FLG_MASK            0x40
#define BA70X_POR_FLG_SHIFT           6
#define BA70X_POR_FLG_N               0//Indicate a POR event has never happened (default)
#define BA70X_POR_FLG                 1//Indicate a POR event has ever happened

#define BA70X_RESET_FLG_MASK          0x20
#define BA70X_RESET_FLGT_SHIFT        5
#define BA70X_RESET_FLG_N             0//Normal (default)
#define BA70X_RESET_FLG               1//RESET function occurred

#define BA70X_ADC_DONE_FLG_MASK       0x10
#define BA70X_ADC_DONE_FLG_SHIFT      4
#define BA70X_ADC_DONE_FLG_N          0
#define BA70X_ADC_DONE_FLG            1//Indicates the ADC conversion has ever been completed in 1-shot mode only

#define BA70X_REGN_OK_FLG_MASK        0x08
#define BA70X_REGN_OK_FLG_SHIFT       3
#define BA70X_REGN_OK_FLG_N           0//Normal (default)
#define BA70X_REGN_OK_FLG             1//REGN_OK signal rising edge detected

#define BA70X_VBAT_PRESENT_FLG_MASK   0x04
#define BA70X_VBAT_PRESENT_FLG_SHIFT  2
#define BA70X_VBAT_PRESENT_FLG_N      0//Normal (default)
#define BA70X_VBAT_PRESENT_FLG        1//VBAT_UVLO signal status changed (for both rising and falling edge)

#define BA70X_VBUS_PRESENT_FLG_MASK   0x02
#define BA70X_VBUS_PRESENT_FLG_SHIFT  1
#define BA70X_VBUS_PRESENT_FLG_N      0//Normal (default)
#define BA70X_VBUS_PRESENT_FLG        1//VBUS_PRESENT status signal changed (for both rising and falling edge)

#define BA70X_VAC_PRESENT_FLG_MASK    0x01
#define BA70X_VAC_PRESENT_FLG_SHIFT   0
#define BA70X_VAC_PRESENT_FLG_N       0//Normal (default)
#define BA70X_VAC_PRESENT_FLG         1//VAC_PRESENT signal rising edge detected

//0B
#define BA70X_REG_0B_7                Reserved

#define BA70X_POR_MASK_MASK           0x40
#define BA70X_POR_MASK_SHIFT          6
#define BA70X_POR_MASK_N              0//Not Masked (default)
#define BA70X_POR_MASK                1//Masked

#define BA70X_RESET_MASK_MASK         0x20
#define BA70X_RESET_MASK_SHIFT        5
#define BA70X_RESET_MASK_N            0//Not Masked (default)
#define BA70X_RESET_MASK              1//Masked

#define BA70X_ADC_DONE_MASK_MASK      0x10
#define BA70X_ADC_DONE_MASK_SHIFT     4
#define BA70X_ADC_DONE_MASK_N         0//Not Masked (default)
#define BA70X_ADC_DONE_MASK           1//Masked

#define BA70X_REGN_OK_MASK_MASK       0x08
#define BA70X_REGN_OK_MASK_SHIFT      3
#define BA70X_REGN_OK_MASK_N          0//Not Masked (default)
#define BA70X_REGN_OK_MASK            1//Masked

#define BA70X_VBAT_PRESENT_MASK_MASK  0x04
#define BA70X_VBAT_PRESENT_MASK_SHIFT 2
#define BA70X_VBAT_PRESENT_MASK_N     0//Not Masked (default)
#define BA70X_VBAT_PRESENT_MASK       1//Masked

#define BA70X_VBUS_PRESENT_MASK_MASK  0x02
#define BA70X_VBUS_PRESENT_MASK_SHIFT 1
#define BA70X_VBUS_PRESENT_MASK_N     0//Not Masked (default)
#define BA70X_VBUS_PRESENT_MASK       1//Masked

#define BA70X_VAC_PRESENT_MASK_MASK   0x01
#define BA70X_VAC_PRESENT_MASK_SHIFT  0
#define BA70X_VAC_PRESENT_MASK_N      0//Not Masked (default)
#define BA70X_VAC_PRESENT_MASK        1//Masked

//0C
#define BA70X_REG_0C_7                Reserved
#define BA70X_REG_0C_6                Reserved

#define BA70X_TSHUT_STAT_MASK         0x20
#define BA70X_TSHUT_STAT_SHIFT        5
#define BA70X_TSHUT_STAT_N            0//Normal (default)
#define BA70X_TSHUT_STAT              1//Device is in TSHUT status

#define BA70X_TSBAT_HOT_STAT_MASK     0x10
#define BA70X_TSBAT_HOT_STAT_SHIFT    4
#define BA70X_TSBAT_HOT_STAT_N        0//Normal (default)
#define BA70X_TSBAT_HOT_STAT          1//Device is in TSBAT_HOT status.Deglitch time 10ms

#define BA70X_TSBAT_COLD_STAT_MASK    0x08
#define BA70X_TSBAT_COLD_STAT_SHIFT   3
#define BA70X_TSBAT_COLD_STAT_N       0//Normal (default)
#define BA70X_TSBAT_COLD_STAT         1//Device is in TSBAT_COLD status.Deglitch time 10ms

#define BA70X_TSBUS_FLT_STAT_MASK     0x04
#define BA70X_TSBUS_FLT_STAT_SHIFT    2
#define BA70X_TSBUS_FLT_STAT_N        0//Normal (default)
#define BA70X_TSBUS_FLT_STAT          1//Device is in TSBUS_FLT status.Deglitch time 10ms

#define BA70X_VBUS_OVP_STAT_MASK      0x02
#define BA70X_VBUS_OVP_STAT_SHIFT     1
#define BA70X_VBUS_OVP_STAT_N         0//Normal (default)
#define BA70X_VBUS_OVP_STAT           1//Device is in VBUS_OVP status

#define BA70X_VAC_OVP_STAT_MASK       0x01
#define BA70X_VAC_OVP_STAT_SHIFT      0
#define BA70X_VAC_OVP_STAT_N          0//Normal (default)
#define BA70X_VAC_OVP_STAT            1//Device is in VAC_OVP status

//0D
#define BA70X_REG_0D_7                Reserved

#define BA70X_WD_TIMEOUT_FLG_MASK     0x40
#define BA70X_WD_TIMEOUT_FLG_SHIFT    6
#define BA70X_WD_TIMEOUT_FLG_N        0//Normal (default)
#define BA70X_WD_TIMEOUT_FLG          1//Device has ever been in WD_TIMEOUT status

#define BA70X_TSHUT_FLG_MASK          0x20
#define BA70X_TSHUT_FLG_SHIFT         5
#define BA70X_TSHUT_FLG_N             0//Normal (default)
#define BA70X_TSHUT_FLG               1//Device has ever been in TSHUT status

#define BA70X_TSBAT_HOT_FLG_MASK      0x10
#define BA70X_TSBAT_HOT_FLG_SHIFT     4
#define BA70X_TSBAT_HOT_FLG_N         0//Normal (default)
#define BA70X_TSBAT_HOT_FLG           1//Device has ever been in TSBAT_HOT status

#define BA70X_TSBAT_COLD_FLG_MASK     0x08
#define BA70X_TSBAT_COLD_FLG_SHIFT    3
#define BA70X_TSBAT_COLD_FLG_N        0//Normal (default)
#define BA70X_TSBAT_COLD_FLG          1//Device has ever been in TSBAT_COLD status

#define BA70X_TSBUS_FLT_FLG_MASK      0x04
#define BA70X_TSBUS_FLT_FLG_SHIFT     2
#define BA70X_TSBUS_FLT_FLG_N         0//Normal (default)
#define BA70X_TSBUS_FLT_FLG           1//Device has ever been in TSBUS_FLT status

#define BA70X_VBUS_OVP_FLG_MASK       0x02
#define BA70X_VBUS_OVP_FLG_SHIFT      1
#define BA70X_VBUS_OVP_FLG_N          0//Normal (default)
#define BA70X_VBUS_OVP_FLG            1//Device has ever been in VBUS_OVP status
 
#define BA70X_VAC_OVP_FLG_MASK        0x01
#define BA70X_VAC_OVP_FLG_SHIFT       0
#define BA70X_VAC_OVP_FLG_N           0//Normal (default)
#define BA70X_VAC_OVP_FLG             1//Device has ever been in VAC_OVP status

//0E
#define BA70X_REG_0E_7                Reserved

#define BA70X_WD_TIMEOUT_MASK_MASK    0x40
#define BA70X_WD_TIMEOUT_MASK_SHIFT   6
#define BA70X_WD_TIMEOUT_MASK_N       0//Not Masked (default)
#define BA70X_WD_TIMEOUT_MASK         1//Masked

#define BA70X_TSHUT_MASK_MASK         0x20
#define BA70X_TSHUT_MASK_SHIFT        5
#define BA70X_TSHUT_MASK_N            0//Not Masked (default)
#define BA70X_TSHUT_MASK              1//Masked

#define BA70X_TSBAT_HOT_MASK_MASK     0x10
#define BA70X_TSBAT_HOT_MASK_SHIFT    4
#define BA70X_TSBAT_HOT_MASK_N        0//Not Masked (default)
#define BA70X_TSBAT_HOT_MASK          1//Masked

#define BA70X_TSBAT_COLD_MASK_MASK    0x08
#define BA70X_TSBAT_COLD_MASK_SHIFT   3
#define BA70X_TSBAT_COLD_MASK_N       0//Not Masked (default)
#define BA70X_TSBAT_COLD_MASK         1//Masked

#define BA70X_TSBUS_FLT_MASK_MASK     0x04
#define BA70X_TSBUS_FLT_MASK_SHIFT    2
#define BA70X_TSBUS_FLT_MASK_N        0//Not Masked (default)
#define BA70X_TSBUS_FLT_MASK          1//Masked

#define BA70X_VBUS_OVP_MASK_MASK      0x02
#define BA70X_VBUS_OVP_MASK_SHIFT     1
#define BA70X_VBUS_OVP_MASK_N         0//Not Masked (default)
#define BA70X_VBUS_OVP_MASK           1//Masked

#define BA70X_VAC_OVP_MASK_MASK       0x01
#define BA70X_VAC_OVP_MASK_SHIFT      0
#define BA70X_VAC_OVP_MASK_N          0//Not Masked (default)
#define BA70X_VAC_OVP_MASK            1//Masked

//0F

*/




























#endif


