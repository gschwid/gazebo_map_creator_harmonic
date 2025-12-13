#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>
#include <string>
#include <gz/common/Console.hh>

namespace map_creator {

    class MapCreator:
        public gz::sim::System,
        public gz::sim::ISystemPostUpdate
        {
        public:
            void PostUpdate(const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) override;
        };
}   