#include "map_parser.hpp"

GZ_ADD_PLUGIN(
    map_parser::MapParser,
    gz::sim::System,
    map_parser::MapParser::ISystemPostUpdate);

using namespace map_parser;

void MapParser::PostUpdate(const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager & _ecem)
{
    this->getDimensions(_ecem);

            //     if (_geometry->Data().Type() == sdf::GeometryType::PLANE){
            //     gzmsg << "Found Model Entity [" << _entity << "] with Name: " 
            //     << " and Geometry: " << _geometry->Data().PlaneShape()->Size().X() << std::endl;
            //       return true;
            //     }
            //     return false;

    // std::string msg = "Hello, world! Simulation is ";
    // if (!_info.paused)
    // msg += "not ";
    // msg += "paused.";

    // // Messages printed with gzmsg only show when running with verbosity 3 or
    // // higher (i.e. gz sim -v 3)
    // gzmsg << msg << std::endl;
}

void MapParser::getDimensions(const gz::sim::EntityComponentManager & _ecem) {
    int smallest_x;
    int largest_x;
    int smallest_y;
    int largest_y;

    _ecem.Each<gz::sim::components::Collision, gz::sim::components::Geometry>(
            [&smallest_x, &largest_x, &smallest_y, &largest_y, &_ecem](const gz::sim::Entity & _entity, const gz::sim::components::Collision * _model, const gz::sim::components::Geometry *_geom) -> bool {

                // Get world pose of each collision object
                gz::math::Pose3d worldPose = gz::sim::worldPose( _entity, _ecem);
                gzmsg << "X: " << worldPose.Pos().X() << " Y: " << worldPose.Pos().Y() << " Z: " << worldPose.Pos().Z() << std::endl;

                // Get the size depending on what type of object we are looking at
                gz::math::Vector3d scale;

                if (_geom->Data().Type() == sdf::GeometryType::BOX) {
                    scale = _geom->Data().BoxShape()->Size();
                }

                else if (_geom->Data().Type() == sdf::GeometryType::CYLINDER) {
                    scale.X() = _geom->Data().CylinderShape()->Radius() * 2;
                    scale.Y() = scale.X();
                    scale.Z() = _geom->Data().CylinderShape()->Length();
                }

                else if (_geom->Data().Type() == sdf::GeometryType::SPHERE) {
                    scale.X() = _geom->Data().SphereShape()->Radius() * 2;
                    scale.Y() = scale.X();
                    scale.Z() = scale.X();
                }

                else if (_geom->Data().Type() == sdf::GeometryType::PLANE) {
                    scale.X() = _geom->Data().PlaneShape()->Size().X();
                    scale.Y() = _geom->Data().PlaneShape()->Size().Y();
                    scale.Z() = 0;
                }

                else if (_geom->Data().Type() == sdf::GeometryType::CYLINDER) {
                    scale.X() = _geom->Data().CylinderShape()->Radius() * 2;
                    scale.Y() =  scale.X();
                    scale.Z() = scale.X();
                }

                gzmsg << "Scale X: " << scale.X() << " Scale Y: " << scale.Y() << " Scale Z: " << scale.Z() << std::endl;
                
                return true;
            }
            );
    }