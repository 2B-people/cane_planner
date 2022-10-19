#include <plan_env/edt_compress.h>
#include <plan_env/sdf_map.h>

using namespace fast_planner;

namespace cane_planner
{
    EDTCompress::EDTCompress()
    {
    }
    EDTCompress::~EDTCompress()
    {
    }

    void EDTCompress::init(ros::NodeHandle &nh)
    {
        md_2d_.reset(new MapData);
        mp_2d_.reset(new MapParam);
    }

    void EDTCompress::setMap(shared_ptr<SDFMap> &map)
    {
        this->sdf_map_ = map;
    }

} // namespace cane_planner
