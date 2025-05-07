#ifndef FAST_RNRR_NONRIGIDREG_H
#define FAST_RNRR_NONRIGIDREG_H

#include <string>
#include <vector>

namespace fast_rnrr {

// 前向声明
class NonRigidRegImpl;

/**
 * @brief 非刚性配准库的主要接口类
 */
class NonRigidReg {
public:
    /**
     * @brief 构造函数
     */
    NonRigidReg();
    
    /**
     * @brief 析构函数
     */
    ~NonRigidReg();
    
    /**
     * @brief 配置参数结构体
     */
    struct Parameters {
        // 刚性配准参数
        double alpha = 100.0;
        double beta = 100.0;
        double gamma = 1e8;
        double uni_sample_radio = 5.0;
        
        // 对应点筛选参数
        bool use_distance_reject = true;
        double distance_threshold = 0.05;
        bool use_normal_reject = false;
        double normal_threshold = 3.14159265358979323846 / 3;
        bool use_Dynamic_nu = true;
        
        // 特征点参数
        bool use_landmark = false;
        std::vector<size_t> landmark_src;
        std::vector<size_t> landmark_tar;
        
        // 是否计算误差
        bool calc_gt_err = true;
        
        // 输出参数
        std::string out_gt_file;
    };
    
    /**
     * @brief 设置配准参数
     * @param params 配准参数
     */
    void setParameters(const Parameters& params);
    
    /**
     * @brief 获取当前配准参数
     * @return 当前配准参数
     */
    Parameters getParameters() const;
    
    /**
     * @brief 执行非刚性配准
     * @param src_file 源网格文件路径
     * @param tar_file 目标网格文件路径
     * @param out_file 输出结果文件路径
     * @param landmark_file 特征点文件路径（可选）
     * @return 是否成功
     */
    bool registerMeshes(const std::string& src_file, 
                        const std::string& tar_file, 
                        const std::string& out_file,
                        const std::string& landmark_file = "");
    
    /**
     * @brief 获取配准执行时间信息
     * @return 包含各阶段执行时间的字符串
     */
    std::string getTimingInfo() const;

private:
    NonRigidRegImpl* impl_; // PIMPL 模式实现
};

} // namespace fast_rnrr

#endif // FAST_RNRR_NONRIGIDREG_H
