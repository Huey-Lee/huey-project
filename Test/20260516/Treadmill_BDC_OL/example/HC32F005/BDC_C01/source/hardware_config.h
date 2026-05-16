/* hardware_config.h — 模拟前端 / ADC 物理量标定（与原理图一致，单点修改） */
#ifndef HARDWARE_CONFIG_H_
#define HARDWARE_CONFIG_H_

/* MVoltage±：RB7/RB8(各 255k) 串联后与 RB5 或 RB10(5.1k) 分压
 * Ratio = (510k + 5.1k) / 5.1k = 101.0 */
#define V_BUS_DIVIDER_RATIO    (101.0f)

#ifndef VOLT_DIV_RATIO
#define VOLT_DIV_RATIO         (V_BUS_DIVIDER_RATIO) /* 与部分文档/旧命名兼容 */
#endif

/* HC32F005 DVCC = VCC_5V，ADC 参考按 5.0 V（与旧码 4.95 V 量纲一致；若有实测 AVDD 可微调） */
#define V_REF_ADC              (5.0f)

/* 与旧文档/别名字段对齐（物理含义同 V_REF_ADC） */
#ifndef MCU_VREF_V
#define MCU_VREF_V             (V_REF_ADC)
#endif

#define ADC_RESOLUTION         (4096.0f)  /* 12 位满量程计数 */

/*──────── 母线侧电压 ↔ ADC 计数（M+、M− 经 101:1 到 MCU，与 motor.h CALC_ADC_2_VOLTAGE 同源）────────
 * 每 1 个 ADC LSB 对应母线伏特：V_REF × 分压比 / 4096 → 约 0.12329 V（5.0×101/4096）。
 * 用途：GET_PHYSICAL_V()、电压闭环 Vcmd_phys/Vbus_phys 等。 */
#ifndef VOLT_PER_ADC_TICK
#define VOLT_PER_ADC_TICK      ((V_REF_ADC) * (V_BUS_DIVIDER_RATIO) / (ADC_RESOLUTION))
#endif
/* 物理常数（原理图决定；数值取自本文件 V_REF_ADC / V_BUS_DIVIDER_RATIO / ADC_RESOLUTION） */
#define PHYS_VREF_V            (V_REF_ADC)
#define PHYS_DIVIDER_RATIO     (V_BUS_DIVIDER_RATIO)
#define PHYS_ADC_RES           (ADC_RESOLUTION)
/* 母线 1V → ADC 计数（≈4096/(Vref×分压比)；与电网 110/220 无关） */
#define PHYS_VOLT_SCALE        ((PHYS_ADC_RES) / ((PHYS_VREF_V) * (PHYS_DIVIDER_RATIO)))

#ifndef GET_PHYSICAL_V
#define GET_PHYSICAL_V(adc_)   ((float)(adc_) * (VOLT_PER_ADC_TICK))
#endif

/* 占空：Duty_ticks = Target_Volts × PHYS_VOLT_SCALE × Period / Realtime_Bus_ADC */
#ifndef MOTOR_BUS_ADC_SCALE_COUNTS_PER_VOLT
#define MOTOR_BUS_ADC_SCALE_COUNTS_PER_VOLT  (PHYS_VOLT_SCALE)
#endif

/* RB18 康铜丝，原理图标称 30 mΩ */
#define CURRENT_SENSE_RES      (0.030f)

/* MCurrent_Back → ADC(AIN0) 经保护/RC，电机板图中无外部电流运放 → 等效增益 1 */
#define CURRENT_AMP_GAIN       (1.0f)

#endif /* HARDWARE_CONFIG_H_ */
