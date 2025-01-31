#![no_std]

pub mod cdcg;
pub mod gpio;
pub mod gpio_miwu;
pub mod i2c;
pub mod miwu;
pub mod espi;

pub use npcx490m_pac as pac;

#[non_exhaustive]
#[derive(Debug, Clone, Default)]
pub struct Config {
    pub cdcg: cdcg::Config,
}

embassy_hal_internal::peripherals!(
    PA02, PA03, PA04, PA09, PA10, PA11, PA12, PB02, PB03, PB04, PB05, PB06, PB07, PB08, PB09, PB10, PB11, PB12, PC01,
    PC02, PC03, PC04, PC05, PC06, PC07, PC08, PC09, PC10, PC11, PC12, PD02, PD03, PD04, PD05, PD06, PD07, PD08, PD09,
    PD10, PD11, PE02, PE03, PE04, PE05, PE06, PE07, PE08, PE09, PE10, PE11, PF02, PF03, PF04, PF05, PF06, PF07, PF08,
    PF09, PF10, PF11, PF12, PG02, PG03, PG04, PG05, PG06, PG07, PG08, PG09, PG10, PG11, PG12, PH01, PH02, PH03, PH04,
    PH05, PH06, PH07, PH08, PH09, PH10, PH11, PI01, PI02, PI03, PI04, PI05, PI06, PI07, PI08, PI09, PI10, PI11, PI12,
    PJ01, PJ02, PJ03, PJ04, PJ05, PJ06, PJ07, PJ08, PJ09, PJ10, PJ11, PK01, PK02, PK03, PK04, PK05, PK06, PK07, PK08,
    PK09, PK10, PK11, PK12, PL01, PL02, PL03, PL05, PL06, PL07, PL08, PL09, PL10, PL11, PL12, PM02, PM01, PM04, PM05,
    PM06, PM07, PM11, PM12, MIWU0_10, MIWU0_11, MIWU0_12, MIWU0_13, MIWU0_14, MIWU0_15, MIWU0_16, MIWU0_17, MIWU0_20,
    MIWU0_21, MIWU0_22, MIWU0_23, MIWU0_24, MIWU0_25, MIWU0_26, MIWU0_27, MIWU0_30, MIWU0_31, MIWU0_32, MIWU0_33,
    MIWU0_34, MIWU0_35, MIWU0_36, MIWU0_37, MIWU0_40, MIWU0_41, MIWU0_42, MIWU0_43, MIWU0_44, MIWU0_45, MIWU0_46,
    MIWU0_47, MIWU0_50, MIWU0_51, MIWU0_52, MIWU0_53, MIWU0_54, MIWU0_55, MIWU0_56, MIWU0_57, MIWU0_60, MIWU0_61,
    MIWU0_62, MIWU0_63, MIWU0_64, MIWU0_65, MIWU0_66, MIWU0_67, MIWU0_70, MIWU0_71, MIWU0_72, MIWU0_73, MIWU0_74,
    MIWU0_75, MIWU0_76, MIWU0_77, MIWU0_80, MIWU0_81, MIWU0_82, MIWU0_83, MIWU0_84, MIWU0_85, MIWU0_86, MIWU0_87,
    MIWU1_10, MIWU1_11, MIWU1_12, MIWU1_13, MIWU1_14, MIWU1_15, MIWU1_16, MIWU1_17, MIWU1_20, MIWU1_21, MIWU1_22,
    MIWU1_23, MIWU1_24, MIWU1_25, MIWU1_26, MIWU1_27, MIWU1_30, MIWU1_31, MIWU1_32, MIWU1_33, MIWU1_34, MIWU1_35,
    MIWU1_36, MIWU1_37, MIWU1_40, MIWU1_41, MIWU1_42, MIWU1_43, MIWU1_44, MIWU1_45, MIWU1_46, MIWU1_47, MIWU1_50,
    MIWU1_51, MIWU1_52, MIWU1_53, MIWU1_54, MIWU1_55, MIWU1_56, MIWU1_57, MIWU1_60, MIWU1_61, MIWU1_62, MIWU1_63,
    MIWU1_64, MIWU1_65, MIWU1_66, MIWU1_67, MIWU1_70, MIWU1_71, MIWU1_72, MIWU1_73, MIWU1_74, MIWU1_75, MIWU1_76,
    MIWU1_77, MIWU1_80, MIWU1_81, MIWU1_82, MIWU1_83, MIWU1_84, MIWU1_85, MIWU1_86, MIWU1_87, MIWU2_10, MIWU2_11,
    MIWU2_12, MIWU2_13, MIWU2_14, MIWU2_15, MIWU2_16, MIWU2_17, MIWU2_20, MIWU2_21, MIWU2_22, MIWU2_23, MIWU2_24,
    MIWU2_25, MIWU2_26, MIWU2_27, MIWU2_30, MIWU2_31, MIWU2_32, MIWU2_33, MIWU2_34, MIWU2_35, MIWU2_36, MIWU2_37,
    MIWU2_40, MIWU2_41, MIWU2_42, MIWU2_43, MIWU2_44, MIWU2_45, MIWU2_46, MIWU2_47, MIWU2_50, MIWU2_51, MIWU2_52,
    MIWU2_53, MIWU2_54, MIWU2_55, MIWU2_56, MIWU2_57, MIWU2_60, MIWU2_61, MIWU2_62, MIWU2_63, MIWU2_64, MIWU2_65,
    MIWU2_66, MIWU2_67, MIWU2_70, MIWU2_71, MIWU2_72, MIWU2_73, MIWU2_74, MIWU2_75, MIWU2_76, MIWU2_77, MIWU2_80,
    MIWU2_81, MIWU2_82, MIWU2_83, MIWU2_84, MIWU2_85, MIWU2_86, MIWU2_87, SMB0, SMB1, SMB2, SMB3, SMB4, SMB5, SMB6,
    SMB7, ESPI
);

#[macro_export]
macro_rules! bind_interrupts {
    ($vis:vis struct $name:ident {
        $(
            $(#[cfg($cond_irq:meta)])?
            $irq:ident => $(
                $(#[cfg($cond_handler:meta)])?
                $handler:ty
            ),*;
        )*
    }) => {
        #[derive(Copy, Clone)]
        $vis struct $name;

        $(
            #[allow(non_snake_case)]
            #[no_mangle]
            $(#[cfg($cond_irq)])?
            unsafe extern "C" fn $irq() {
                $(
                    $(#[cfg($cond_handler)])?
                    <$handler as $crate::interrupt::typelevel::Handler<$crate::interrupt::typelevel::$irq>>::on_interrupt();

                )*
            }

            $(#[cfg($cond_irq)])?
            $crate::bind_interrupts!(@inner
                $(
                    $(#[cfg($cond_handler)])?
                    unsafe impl $crate::interrupt::typelevel::Binding<$crate::interrupt::typelevel::$irq, $handler> for $name {}
                )*
            );
        )*
    };
    (@inner $($t:tt)*) => {
        $($t)*
    }
}

/// Marker struct for LPC mode
// marked non-exhaustive to ensure the user can't create one from nothing
#[non_exhaustive]
#[derive(Debug, Copy, Clone)]
pub struct Lpc {}

/// Marker struct for ESpi mode
// marked non-exhaustive to ensure the user can't create one from nothing
#[non_exhaustive]
#[derive(Debug, Copy, Clone)]
pub struct ESpi {}

fn init(config: Config) -> Peripherals {
    cdcg::init_clocks(config.cdcg);

    Peripherals::take()
}

pub fn init_lpc(config: Config) -> (Peripherals, Lpc) {
    let per = init(config);

    // We still have control over all peripherals, so this is safe to do outside a critical section
    unsafe { crate::pac::Sysconfig::steal() }.devcnt().modify(|r, w| {
        assert!(r.hif_typ_sel().bits() == 0 || r.hif_typ_sel().bits() == 1);
        unsafe { w.hif_typ_sel().bits(1) }
    });

    (per, Lpc {})
}

pub fn init_espi(config: Config) -> (Peripherals, ESpi) {
    let per = init(config);

    // We still have control over all peripherals, so this is safe to do outside a critical section
    unsafe { crate::pac::Sysconfig::steal() }.devcnt().modify(|r, w| {
        assert!(r.hif_typ_sel().bits() == 0 || r.hif_typ_sel().bits() == 2);
        unsafe { w.hif_typ_sel().bits(2) }
    });

    (per, ESpi {})
}

embassy_hal_internal::interrupt_mod!(
    KBS,
    PM_OBE,
    PECI,
    WKINTD_0,
    DP80,
    WKINTA_0,
    SMB7,
    MFT16_1,
    ADC_IREF,
    WKINTE_0,
    GDMA1,
    SMB0,
    SMB1,
    WKINTC_0,
    SMB6,
    ITIM32_3,
    ESPI_SHI,
    SMB4,
    SMB5,
    PS2,
    ADC_EREF,
    MFT16_2,
    SHM,
    KBC_IBF,
    PM_IBF,
    ITIM32_2,
    ITIM32_1,
    I3C1_MDMA5,
    FLM,
    WKINTB_0,
    CR_UART2_MDMA2,
    CR_UART1_MDMA1,
    RNG,
    WKINTF_0,
    SMB2,
    SMB3,
    CR_UART3_MDMA3,
    CR_UART4_MDMA4,
    PKA,
    MFT16_3,
    WKINTG_0,
    ITIM32_4,
    ITIM32_5,
    ITIM32_6,
    WKINTH_0,
    WKINTA_1,
    WKINTB_1,
    WKINTC_1,
    WKINTD_1,
    WKINTE_1,
    WKINTF_1,
    WKINTG_1,
    WKINTH_1,
    WKINTG_2,
    KBC_OBE,
    SPIP,
    WKINTF_2,
    WKINTA_2,
    WKINTB_2,
    WKINTC_2,
    WKINTD_2,
    WKINTE_2,
    GDMA2,
    I3C2_MDMA6,
    I3C3_MDMA7,
    ITIM64,
    WKINTH_2,
);
