/* hardware_config.h — 模拟前端 / ADC 物理量标定（与原理图一致，单点修改） */
#ifndef HARDWARE_CONFIG_H_
#define HARDWARE_CONFIG_H_

/* MVoltage±：RB7/RB8(各 255k) 串联后与 RB5 或 RB10(5.1k) 分压
 * Ratio = (510k + 5.1k) / 5.1k = 101.0 */
#define V_BUS_DIVIDER_RATIO    (101.0f)

/* HC32F005 DVCC = VCC_5V，ADC 参考按 5.0 V（与旧码 4.95 V 量纲一致；若有实测 AVDD 可微调） */
#define V_REF_ADC              (5.0f)

#define ADC_RESOLUTION         (4096.0f)  /* 12 位满量程计数 */

/* M+ / M− 分压到 ADC：每 1 个 LSB 对应母线侧电压（V），与 CALC_ADC_2_VOLTAGE 一致 */
#ifndef VOLT_PER_ADC_TICK
#define VOLT_PER_ADC_TICK   ((V_REF_ADC) * (V_BUS_DIVIDER_RATIO) / (ADC_RESOLUTION))
#endif
#ifndef GET_PHYSICAL_V
#define GET_PHYSICAL_V(adc_)  ((float)(adc_) * (VOLT_PER_ADC_TICK))
#endif

/* RB18 康铜丝，原理图标称 30 mΩ */
#define CURRENT_SENSE_RES      (0.030f)

/* MCurrent_Back → ADC(AIN0) 经保护/RC，电机板图中无外部电流运放 → 等效增益 1 */
#define CURRENT_AMP_GAIN       (1.0f)

#endif /* HARDWARE_CONFIG_H_ */
