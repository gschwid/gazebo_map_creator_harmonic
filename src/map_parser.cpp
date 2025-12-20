#include "map_parser.hpp"

GZ_ADD_PLUGIN(
    map_parser::MapParser,
    gz::sim::System,
    map_parser::MapParser::ISystemPostUpdate);

using namespace map_parser;

void MapParser::PostUpdate(const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager & _ecem)
{
    _ecem.Each<gz::sim::components::Collision, gz::sim::components::Geometry>(
            [&](const gz::sim::Entity & _entity, const gz::sim::components::Collision * _model, const gz::sim::components::Geometry *_geometry) -> bool {

                if (_geometry->Data().Type() == sdf::GeometryType::PLANE){
                gzmsg << "Found Model Entity [" << _entity << "] with Name: " 
                << " and Geometry: " << _geometry->Data().PlaneShape()->Size().X() << std::endl;
                  return true;
                }
                return false;
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