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

namespace map_parser {

    class MapParser:
        public gz::sim::System,
        public gz::sim::ISystemPostUpdate
        {
        public:
            void PostUpdate(const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) override;

            void getDimensions(const gz::sim::EntityComponentManager & _ecem);
            void getObstacles(const gz::sim::EntityComponentManager & _ecem);
        };
}   