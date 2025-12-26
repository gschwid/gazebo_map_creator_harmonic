#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>
#include <string>
#include <gz/common/Console.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/AxisAlignedBox.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Util.hh>
#include <sdf/Box.hh>
#include <sdf/Sphere.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Geometry.hh>
#include <sdf/Plane.hh>
#include <gz/sim/Actor.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/AxisAlignedBox.hh>
#include <limits>
#include <math.h>
#include <vector>
#include <gz/transport.hh>
#include <gz/msgs.hh>
#include <gz/msgs/occupancy_grid.pb.h>

namespace map_parser {

    class MapParser:
        public gz::sim::System,
        public gz::sim::ISystemPostUpdate,
        public gz::sim::ISystemPreUpdate,
        public gz::sim::ISystemConfigure
        {
        public:
            void PostUpdate(const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) override;
            void PreUpdate(const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;
            void Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventMgr) override;
            void getDimensions(gz::sim::EntityComponentManager & _ecem);
            void getObstacles(const gz::sim::EntityComponentManager & _ecem);
        
        private:
            bool getOccupancyGridService(
                const gz::msgs::Empty &_req,
                const gz::msgs::OccupancyGrid &_res);
            bool size_init = false;
            gz::transport::Node node;
            std::string service = "/map_parser/occupancy_grid";
            std::vector<std::vector<int>> occupancy_grid;
            u_int32_t grid_width = -1;
            u_int32_t grid_height = -1;
            double GRID_SIZE = 0.05;
            int PADDING = 10;
        };
}   