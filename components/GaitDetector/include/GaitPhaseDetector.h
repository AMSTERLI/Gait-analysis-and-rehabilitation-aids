#ifndef GAIT_PHASE_DETECTOR_H
#define GAIT_PHASE_DETECTOR_H

/* 步态事件枚举 */
enum class GaitEvent { NONE, IC, HR, TO, FA, TBV };

/**
 * @brief 单轴步态相位检测器
 *
 * 通过上一周期积分面积作为阈值，实时给出
 * IC / HR / TO / FA / TBV 五类事件。
 */
class GaitPhaseDetector {
public:
    GaitPhaseDetector();

    /** 连续喂入一个样本，若有事件立即返回，否则返回 GaitEvent::NONE */
    GaitEvent update(float sample);

    /** 复位所有内部状态 */
    void reset();

    /** 供调试：上一完整周期的正 / 负积分 */
    float getPrevPos() const { return prevPos_; }
    float getPrevNeg() const { return prevNeg_; }

private:
    /*—————— 阈值比例常量 ——————*/
    static constexpr float HR_TH  = 0.46f;
    static constexpr float TO_TH  = 0.95f;
    static constexpr float FA_TH  = 0.20f;
    static constexpr float TBV_TH = 0.73f;

    /*—————— 本周期积分累计 ——————*/
    float posSum_{}, negSum_{}, cumPos_{}, cumNeg_{};

    /*—————— 上一周期积分 ——————*/
    float prevPos_{}, prevNeg_{};

    /*—————— 阈值事件触发标志 ——————*/
    bool hrDone_{}, toDone_{}, faDone_{}, tbvDone_{};
};

#endif /* GAIT_PHASE_DETECTOR_H */
