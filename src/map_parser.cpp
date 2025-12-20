#include "map_parser.hpp"

GZ_ADD_PLUGIN(
    map_parser::MapParser,
    gz::sim::System,
    map_parser::MapParser::ISystemPostUpdate);

using namespace map_parser;

void MapParser::PostUpdate(const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager & _ecem)
{
    _ecem.Each<gz::sim::components::Model, gz::sim::components::Name>(
            [&](const gz::sim::Entity & _entity, const gz::sim::components::Model * _model, const gz::sim::components::Name *_name) -> bool {
                gzmsg << "Found Model Entity [" << _entity << "] with Name: " 
                  << _name->Data() << std::endl;
                  return true;
            }
            );

    // std::string msg = "Hello, world! Simulation is ";
    // if (!_info.paused)
    // msg += "not ";
    // msg += "paused.";

    // // Messages printed with gzmsg only show when running with verbosity 3 or
    // // higher (i.e. gz sim -v 3)
    // gzmsg << msg << std::endl;
}