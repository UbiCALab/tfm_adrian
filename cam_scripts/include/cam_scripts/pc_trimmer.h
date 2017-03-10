#ifndef CAM_SCRIPTS_PC_TRIMMER_H_
#define CAM_SCRIPTS_PC_TRIMMER_H_



#include <cam_scripts/pcstructure.h>


class PCTrimmer
{public:
    typedef bool (*trimFun)(unsigned int x, unsigned int y);

    PCTrimmer(){}
    PCTrimmer(PCStructure pc_structure_in) : pc_structure(pc_structure_in) {}

    void trim(sensor_msgs::PointCloud2&);

    void setTrimmingFunction(trimFun trim_fun_in) { trim_fun = trim_fun_in; }
    template <typename T>
    void setTrimmingFunction(const T& param_in){ trim_fun = AutoTrimFunction(param_in); }
    template <typename T>
    void setTrimmingFunction(const T& x, const T& y){ trim_fun = AutoTrimFunction(x, y); }

    void setStructure(PCStructure pc_structure_in) { pc_structure = pc_structure_in; }

    private:
    trimFun trim_fun;
    PCStructure pc_structure;

};


struct AutoTrimFunction
{
    AutoTrimFunction(const std::vector<idx>& to_trim_in) : to_trim(to_trim_in) {}
    AutoTrimFunction(const std::vector<int>&, const std::vector<int>&);

    bool operator()(unsigned int x, unsigned int y);
private:
    std::vector<idx> to_trim;
};



#endif  // CAM_SCRIPTS_PC_TRIMMER_H_
