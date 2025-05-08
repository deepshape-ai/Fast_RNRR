#include "INonRigidReg.h"
#include "tools/io_mesh.h"
#include "tools/OmpHelper.h"
#include "NonRigidreg.h"
#include <sstream>
#include <algorithm>

namespace fast_rnrr {

// PIMPL 实现类
class NonRigidRegImpl {
public:
    NonRigidRegImpl() : reg_(nullptr), params_() {}
    
    ~NonRigidRegImpl() {
        if (reg_) {
            delete reg_;
            reg_ = nullptr;
        }
    }
    
    bool registerMeshes(const std::string& src_file, 
                        const std::string& tar_file, 
                        const std::string& out_file,
                        const std::string& landmark_file) {
        // 读取网格数据
        Mesh src_mesh;
        Mesh tar_mesh;
        
        if (!read_data(src_file.c_str(), src_mesh) || !read_data(tar_file.c_str(), tar_mesh)) {
            return false;
        }
        
        // 设置参数
        RegParas paras;
        paras.alpha = params_.alpha;
        paras.beta = params_.beta;
        paras.gamma = params_.gamma;
        paras.uni_sample_radio = params_.uni_sample_radio;
        
        paras.use_distance_reject = params_.use_distance_reject;
        paras.distance_threshold = params_.distance_threshold;
        paras.use_normal_reject = params_.use_normal_reject;
        paras.normal_threshold = params_.normal_threshold;
        paras.use_Dynamic_nu = params_.use_Dynamic_nu;
        
        paras.use_landmark = params_.use_landmark;
        std::copy(params_.landmark_src.begin(), params_.landmark_src.end(), paras.landmark_src.begin());
        std::copy(params_.landmark_tar.begin(), params_.landmark_tar.end(), paras.landmark_tar.begin());
        
        paras.calc_gt_err = params_.calc_gt_err;
        paras.out_gt_file = params_.out_gt_file;
        
        // 如果提供了特征点文件，读取特征点
        if (!landmark_file.empty()) {
            paras.use_landmark = true;
            read_landmark(landmark_file.c_str(), paras.landmark_src, paras.landmark_tar);
        }
        
        // 网格缩放
        double scale = mesh_scaling(src_mesh, tar_mesh);
        
        // 创建配准对象
        if (reg_) {
            delete reg_;
        }
        //> wtf, no ()
        reg_ = new NonRigidreg;
        
        // 执行配准
        Timer time;
        Timer::EventID time1, time2, time3, time4, time5;
        
        // 刚性配准初始化
        time1 = time.get_time();
        reg_->rigid_init(src_mesh, tar_mesh, paras);
        time2 = time.get_time();
        
        // 执行刚性配准
        reg_->DoRigid();
        time3 = time.get_time();
        
        // 非刚性配准初始化
        reg_->Initialize();
        time4 = time.get_time();
        reg_->pars_.non_rigid_init_time = time.elapsed_time(time3, time4);
        
        // 执行非刚性配准
        reg_->DoNonRigid();
        time5 = time.get_time();
        
        // 记录时间信息
        timing_info_.rigid_init_time = time.elapsed_time(time1, time2);
        timing_info_.rigid_reg_time = time.elapsed_time(time2, time3);
        timing_info_.nonrigid_init_time = time.elapsed_time(time3, time4);
        timing_info_.nonrigid_reg_time = time.elapsed_time(time4, time5);
        
        // 写入结果
        return write_data(out_file.c_str(), src_mesh, scale);
    }
    
    std::string getTimingInfo() const {
        std::stringstream ss;
        ss << "刚性配准初始化时间: " << timing_info_.rigid_init_time << " 秒\n";
        ss << "刚性配准执行时间: " << timing_info_.rigid_reg_time << " 秒\n";
        ss << "非刚性配准初始化时间: " << timing_info_.nonrigid_init_time << " 秒\n";
        ss << "非刚性配准执行时间: " << timing_info_.nonrigid_reg_time << " 秒\n";
        ss << "总执行时间: " << (timing_info_.rigid_init_time + timing_info_.rigid_reg_time + 
                            timing_info_.nonrigid_init_time + timing_info_.nonrigid_reg_time) << " 秒";
        return ss.str();
    }
    
    NonRigidReg::Parameters params_ {};
    NonRigidreg* reg_ = nullptr;
    
    struct TimingInfo {
        double rigid_init_time = 0.0;
        double rigid_reg_time = 0.0;
        double nonrigid_init_time = 0.0;
        double nonrigid_reg_time = 0.0;
    } timing_info_;
};

// 实现公共接口
NonRigidReg::NonRigidReg() : impl_(new NonRigidRegImpl()) {}

NonRigidReg::~NonRigidReg() {
    if (impl_) {
        delete impl_;
        impl_ = nullptr;
    }
}

void NonRigidReg::setParameters(const Parameters& params) {
    impl_->params_ = params;
}

NonRigidReg::Parameters NonRigidReg::getParameters() const {
    return impl_->params_;
}

bool NonRigidReg::registerMeshes(const std::string& src_file, 
                                const std::string& tar_file, 
                                const std::string& out_file,
                                const std::string& landmark_file) {
    return impl_->registerMeshes(src_file, tar_file, out_file, landmark_file);
}

std::string NonRigidReg::getTimingInfo() const {
    return impl_->getTimingInfo();
}

} // namespace fast_rnrr