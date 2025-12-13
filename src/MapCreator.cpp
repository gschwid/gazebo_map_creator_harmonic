#include "MapCreator.hpp"

GZ_ADD_PLUGIN(
    map_creator::MapCreator,
    gz::sim::System,
    map_creator::MapCreator::ISystemPostUpdate);

using namespace map_creator;

void MapCreator::PostUpdate(const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &/*_ecm*/)
{
    std::string msg = "Hello, world! Simulation is ";
    if (!_info.paused)
    msg += "not ";
    msg += "paused.";

    // Messages printed with gzmsg only show when running with verbosity 3 or
    // higher (i.e. gz sim -v 3)
    gzmsg << msg << std::endl;
}