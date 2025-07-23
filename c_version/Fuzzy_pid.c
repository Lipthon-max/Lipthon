#include "Fuzzy_pid.h"
#include "string.h"

void FuzzyPID_Init(FuzzyPIDController *fpid)
{
    memset(fpid, 0, sizeof(FuzzyPIDController));
    fpid->e_min = -100.0f;
    fpid->e_max = 100.0f;
    fpid->ec_min = -50.0f;
    fpid->ec_max = 50.0f;
    fpid->kp_min = 0.0f;
    fpid->kp_max = 10.0f;
    fpid->ki_min = 0.0f;
    fpid->ki_max = 5.0f;
    fpid->kd_min = 0.0f;
    fpid->kd_max = 2.0f;

    fpid->error_history[0] = 0.0f;
    fpid->error_history[1] = 0.0f;

    // 初始化模糊规则
    int8_t kp_rules[7][7] = {
        // EC: NB    NM    NS    ZO    PS    PM    PL
        {PL, PL, PM, PM, PS, ZO, ZO}, // E: NB
        {PL, PL, PM, PS, PS, ZO, NS}, // E: NM
        {PM, PM, PM, PS, ZO, NS, NS}, // E: NS
        {PM, PM, PS, ZO, NS, NM, NM}, // E: ZO
        {PS, PS, ZO, NS, NS, NM, NM}, // E: PS
        {PS, ZO, NS, NM, NM, NM, NB}, // E: PM
        {ZO, ZO, NM, NM, NM, NB, NB}  // E: PL
    };
    int8_t ki_rules[7][7] = {
        // EC: NB    NM    NS    ZO    PS    PM    PL
        {NB, NB, NM, NM, NS, ZO, ZO}, // E: NB
        {NB, NB, NM, NS, NS, ZO, ZO}, // E: NM
        {NB, NM, NS, NS, ZO, PS, PS}, // E: NS
        {NM, NM, NS, ZO, PS, PM, PM}, // E: ZO
        {NM, NS, ZO, PS, PS, PM, PL}, // E: PS
        {ZO, ZO, PS, PS, PM, PL, PL}, // E: PM
        {ZO, ZO, PS, PM, PM, PL, PL}  // E: PL
    };
    int8_t kd_rules[7][7] = {
        // EC: NB    NM    NS    ZO    PS    PM    PL
        {PS, NS, NB, NB, NB, NM, PS}, // E: NB
        {PS, NS, NB, NM, NM, NS, ZO}, // E: NM
        {ZO, NS, NM, NM, NS, NS, ZO}, // E: NS
        {ZO, NS, NS, NS, NS, NS, ZO}, // E: ZO
        {ZO, ZO, ZO, ZO, ZO, ZO, ZO}, // E: PS
        {PL, NS, PS, PS, PS, PS, PL}, // E: PM
        {PL, PM, PM, PM, PS, PS, PL}  // E: PL
    };
    memcpy(fpid->rule_kp, kp_rules, sizeof(kp_rules));
    memcpy(fpid->rule_ki, ki_rules, sizeof(ki_rules));
    memcpy(fpid->rule_kd, kd_rules, sizeof(kd_rules));
}

void FuzzyPID_SetParameters(FuzzyPIDController *fpid, float Kp, float Ki, float Kd,
                            float e_min, float e_max, float ec_min, float ec_max,
                            float kp_min, float kp_max, float ki_min, float ki_max,
                            float kd_min, float kd_max)
{
    fpid->Kp = Kp;
    fpid->Ki = Ki;
    fpid->Kd = Kd;
    fpid->e_min = e_min;
    fpid->e_max = e_max;
    fpid->ec_min = ec_min;
    fpid->ec_max = ec_max;
    fpid->kp_min = kp_min;
    fpid->kp_max = kp_max;
    fpid->ki_min = ki_min;
    fpid->ki_max = ki_max;
    fpid->kd_min = kd_min;
    fpid->kd_max = kd_max;
}

static float Normalize(float value, float min, float max)
{
    float qvalues = 6.0 * (value - min) / (max - min) - 3.0f;
    return qvalues < -3.0f ? -3.0f : (qvalues > 3.0f ? 3.0f : qvalues);
}
static float Denormalize(float qvalue, float min, float max)
{
    float value = (max - min) * (qvalue + 3) / 6 + min;
    return value < min ? min : (value > max ? max : value);
}
void CalculateMemberships(FuzzyPIDController *fpid, float *e_memship, float *ec_memship)
{
    // 归一化误差和误差变化
    float e_normalized = Normalize(fpid->error_history[0], fpid->e_min, fpid->e_max);
    float ec_normalized = Normalize(fpid->error_history[0] - fpid->error_history[1], fpid->ec_min, fpid->ec_max);

    // 2. 计算误差的隶属度
    for (int i = 0; i < 7; i++)
    {
        e_memship[i] = 0.0f;
    }

    if (e_normalized >= -3.0f && e_normalized < -2.0f)
    {
        e_memship[NB] = -e_normalized - 2.0f;
        e_memship[NM] = e_normalized + 3.0f;
    }
    else if (e_normalized >= -2.0f && e_normalized < -1.0f)
    {
        e_memship[NM] = -e_normalized - 1.0f;
        e_memship[NS] = e_normalized + 2.0f;
    }
    else if (e_normalized >= -1.0f && e_normalized < 0.0f)
    {
        e_memship[NS] = -e_normalized;
        e_memship[ZO] = e_normalized + 1.0f;
    }
    else if (e_normalized >= 0.0f && e_normalized < 1.0f)
    {
        e_memship[ZO] = 1.0f - e_normalized;
        e_memship[PS] = e_normalized;
    }
    else if (e_normalized >= 1.0f && e_normalized < 2.0f)
    {
        e_memship[PS] = 2.0f - e_normalized;
        e_memship[PM] = e_normalized - 1.0f;
    }
    else if (e_normalized >= 2.0f && e_normalized < 3.0f)
    {
        e_memship[PM] = 3.0f - e_normalized;
        e_memship[PL] = e_normalized - 2.0f;
    }
    else if (e_normalized == 3.0f)
    {
        e_memship[PL] = 1.0f;
    }
    else if (e_normalized == -3.0f)
    {
        e_memship[NB] = 1.0f;
    }

    // 3. 计算误差变化的隶属度
    for (int i = 0; i < 7; i++)
    {
        ec_memship[i] = 0.0f;
    }

    if (ec_normalized >= -3.0f && ec_normalized < -2.0f)
    {
        ec_memship[NB] = -ec_normalized - 2.0f;
        ec_memship[NM] = ec_normalized + 3.0f;
    }
    else if (ec_normalized >= -2.0f && ec_normalized < -1.0f)
    {
        ec_memship[NM] = -ec_normalized - 1.0f;
        ec_memship[NS] = ec_normalized + 2.0f;
    }
    else if (ec_normalized >= -1.0f && ec_normalized < 0.0f)
    {
        ec_memship[NS] = -ec_normalized;
        ec_memship[ZO] = ec_normalized + 1.0f;
    }
    else if (ec_normalized >= 0.0f && ec_normalized < 1.0f)
    {
        ec_memship[ZO] = 1.0f - ec_normalized;
        ec_memship[PS] = ec_normalized;
    }
    else if (ec_normalized >= 1.0f && ec_normalized < 2.0f)
    {
        ec_memship[PS] = 2.0f - ec_normalized;
        ec_memship[PM] = ec_normalized - 1.0f;
    }
    else if (ec_normalized >= 2.0f && ec_normalized < 3.0f)
    {
        ec_memship[PM] = 3.0f - ec_normalized;
        ec_memship[PL] = ec_normalized - 2.0f;
    }
    else if (ec_normalized == 3.0f)
    {
        ec_memship[PL] = 1.0f;
    }
    else if (ec_normalized == -3.0f)
    {
        ec_memship[NB] = 1.0f;
    }
}

void FuzzyInference(FuzzyPIDController *fpid, float *e_memship, float *ec_memship, float *kp_act, float *ki_act, float *kd_act)
{
    for (int i = 0; i < 7; i++)
    {
        kp_act[i] = 0.0f;
        ki_act[i] = 0.0f;
        kd_act[i] = 0.0f;
    }
    // 遍历所有规则,计算kpkikd的隶属度
    for (int e_idx = 0; e_idx < 7; e_idx++)
    {
        for (int ec_idx = 0; ec_idx < 7; ec_idx++)
        {
            float activation = (e_memship[e_idx] < ec_memship[ec_idx]) ? e_memship[e_idx] : ec_memship[ec_idx];
            int8_t kp_rule = fpid->rule_kp[e_idx][ec_idx];
            int8_t ki_rule = fpid->rule_ki[e_idx][ec_idx];
            int8_t kd_rule = fpid->rule_kd[e_idx][ec_idx];
            if (activation > 0)
            {
                int kp_output_idx = kp_rule + 3;
                int ki_output_idx = ki_rule + 3;
                int kd_output_idx = kd_rule + 3;
                kp_act[kp_output_idx] = (activation > kp_act[kp_output_idx]) ? activation : kp_act[kp_output_idx];
                ki_act[ki_output_idx] = (activation > ki_act[ki_output_idx]) ? activation : ki_act[ki_output_idx];
                kd_act[kd_output_idx] = (activation > kd_act[kd_output_idx]) ? activation : kd_act[kd_output_idx];
            }
        }
    }
}
// 重心法解模糊
void Defuzzification(float *kp_act, float *ki_act, float *kd_act, float *kp_output, float *ki_output, float *kd_output)
{
    // 初始化输出值
    *kp_output = 0.0f;
    *ki_output = 0.0f;
    *kd_output = 0.0f;
    float kp_weight_sum = 0.0f;
    float ki_weight_sum = 0.0f;
    float kd_weight_sum = 0.0f;
    for (int i = 0; i < 7; i++)
    {
        float value = (float)(i - 3);
        *kp_output += kp_act[i] * value;
        *ki_output += ki_act[i] * value;
        *kd_output += kd_act[i] * value;
        kp_weight_sum += kp_act[i];
        ki_weight_sum += ki_act[i];
        kd_weight_sum += kd_act[i];
    }
    if (kp_weight_sum > 0)
        *kp_output /= kp_weight_sum;
    if (ki_weight_sum > 0)
        *ki_output /= ki_weight_sum;
    if (kd_weight_sum > 0)
        *kd_output /= kd_weight_sum;
}

float FuzzyPID_Compute(FuzzyPIDController *fpid, float setpoint, float actual)
{
    // 计算误差和误差变化率
    float error = setpoint - actual;
    fpid->error_history[2] = fpid->error_history[1];
    fpid->error_history[1] = fpid->error_history[0];
    fpid->error_history[0] = error;
    float error_change = error - fpid->error_history[1];
    // 中间变量
    float e_memship[7], ec_memship[7];
    float kp_act[7], ki_act[7], kd_act[7];
    float kp_output, ki_output, kd_output;
    // 模糊pid核心规则计算
    CalculateMemberships(fpid, e_memship, ec_memship);
    FuzzyInference(fpid, e_memship, ec_memship, kp_act, ki_act, kd_act);
    Defuzzification(kp_act, ki_act, kd_act, &kp_output, &ki_output, &kd_output);
    float kp_adj = Denormalize(kp_output, fpid->kp_min, fpid->kp_max);
    float ki_adj = Denormalize(ki_output, fpid->ki_min, fpid->ki_max);
    float kd_adj = Denormalize(kd_output, fpid->kd_min, fpid->kd_max);

    fpid->Kp = kp_adj;
    fpid->Ki = ki_adj;
    fpid->Kd = kd_adj;

    float increment = fpid->Kp * (error - fpid->error_history[1]) +
                      fpid->Ki * (error) +
                      fpid->Kd * (error - 2 * fpid->error_history[1] + fpid->error_history[2]);
    static float output = 0.0f;
    output += increment;
    if (output > 500.0f)
        output = 500.0f;
    if (output < -500.0f)
        output = -500.0f;
    return output;
}
