#include "map_parser.hpp"

GZ_ADD_PLUGIN(
    map_parser::MapParser,
    gz::sim::System,
    map_parser::MapParser::ISystemPostUpdate,
    map_parser::MapParser::ISystemPreUpdate,
    map_parser::MapParser::ISystemConfigure);

using namespace map_parser;

bool MapParser::getOccupancyGridService(
    const gz::msgs::Empty &_req,
    gz::msgs::OccupancyGrid &_res)
{
    if (!this->size_init)
    {
        gzerr << "Occupancy grid has not been initialized yet." << std::endl;
        return false;
    }
    else
    {
        _res.mutable_info()->set_height(this->grid_height);
        _res.mutable_info()->set_width(this->grid_width);
        _res.mutable_info()->set_resolution(this->GRID_SIZE);
        _res.mutable_info()->mutable_origin()->mutable_position()->set_x(0.0); // These need to be updated depending on the map, I dont do that right now.
        _res.mutable_info()->mutable_origin()->mutable_position()->set_y(0.0);
        _res.set_data(this->occupancy_grid.data(), this->occupancy_grid.size());
        gzmsg << "Sending occupancy grid of size: " << this->occupancy_grid.size() << std::endl;

        return true;
    }
}

void MapParser::Configure(const gz::sim::Entity &_entity,
                          const std::shared_ptr<const sdf::Element> &_sdf,
                          gz::sim::EntityComponentManager &_ecm,
                          gz::sim::EventManager &_eventMgr)
{
    if (!node.Advertise(service, &MapParser::getOccupancyGridService, this))
    {
        std::cerr << "Error advertising service [" << service << "]" << std::endl;
        return;
    }
}

void MapParser::PreUpdate(const gz::sim::UpdateInfo &_info,
                          gz::sim::EntityComponentManager &_ecem)
{
    if (!this->size_init)
    {
        this->getDimensions(_ecem);
    }
}

void MapParser::PostUpdate(const gz::sim::UpdateInfo &_info,
                           const gz::sim::EntityComponentManager &_ecem)
{
    // if (true)
    // {
    //     this->getDimensions(_ecem);
    // }

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

    // Messages printed with gzmsg only show when running with verbosity 3 or
    // higher (i.e. gz sim -v 3)
    // gzmsg << msg << std::endl;
}

void MapParser::getObstacles(const gz::sim::EntityComponentManager &_ecem)
{

    _ecem.Each<gz::sim::components::Collision, gz::sim::components::Geometry>(
        [&_ecem](const gz::sim::Entity &_entity, const gz::sim::components::Collision *_model, const gz::sim::components::Geometry *_geom) -> bool
        {
            // Get world pose of each collision object
            gz::math::Pose3d worldPose = gz::sim::worldPose(_entity, _ecem);
            gzmsg << "X: " << worldPose.Pos().X() << " Y: " << worldPose.Pos().Y() << " Z: " << worldPose.Pos().Z() << std::endl;

            // Get the size depending on what type of object we are looking at
            gz::math::Vector3d scale;

            if (_geom->Data().Type() == sdf::GeometryType::BOX)
            {
                scale = _geom->Data().BoxShape()->Size();
            }

            else if (_geom->Data().Type() == sdf::GeometryType::CYLINDER)
            {
                scale.X() = _geom->Data().CylinderShape()->Radius() * 2;
                scale.Y() = scale.X();
                scale.Z() = _geom->Data().CylinderShape()->Length();
            }

            else if (_geom->Data().Type() == sdf::GeometryType::SPHERE)
            {
                scale.X() = _geom->Data().SphereShape()->Radius() * 2;
                scale.Y() = scale.X();
                scale.Z() = scale.X();
            }

            else if (_geom->Data().Type() == sdf::GeometryType::PLANE)
            {
                scale.X() = _geom->Data().PlaneShape()->Size().X();
                scale.Y() = _geom->Data().PlaneShape()->Size().Y();
                scale.Z() = 0;
            }

            else if (_geom->Data().Type() == sdf::GeometryType::CYLINDER)
            {
                scale.X() = _geom->Data().CylinderShape()->Radius() * 2;
                scale.Y() = scale.X();
                scale.Z() = scale.X();
            }

            gzmsg << "Scale X: " << scale.X() << " Scale Y: " << scale.Y() << " Scale Z: " << scale.Z() << std::endl;

            return true;
        });
}

void MapParser::getDimensions(gz::sim::EntityComponentManager &_ecem)
{

    // Setup references to be used as biggest and smallest
    double max = std::numeric_limits<double>::max();
    double min = std::numeric_limits<double>::lowest();
    gz::math::Vector3d smallest(max, max, max);
    gz::math::Vector3d biggest(min, min, min);
    bool boxes_exist = true;

    // Loop through all AABB componenets and find the min and max values
    _ecem.Each<gz::sim::components::Model, gz::sim::components::Name>(
        [&smallest, &biggest, &_ecem, &boxes_exist](const gz::sim::Entity &_entity, const gz::sim::components::Model *model, const gz::sim::components::Name *name) -> bool
        {
            // Check if AABB box exists for this entity
            auto found_box = _ecem.Component<gz::sim::components::AxisAlignedBox>(_entity);
            if (found_box)
            {
                // Dont want to include ground plane because its huge.
                if (name->Data() != "ground_plane")
                {
                    // Update values if the bounding box exists.
                    gz::math::AxisAlignedBox box = found_box->Data();
                    gz::math::Vector3d cur_min = box.Min();
                    gz::math::Vector3d cur_max = box.Max();
                    std::cout << "Name: " << name->Data() << std::endl;
                    std::cout << "cur_x_min " << cur_min.X() << " cur_y_min " << cur_min.Y() << std::endl;
                    std::cout << "cur_x_max " << cur_max.X() << " cur_y_max " << cur_max.Y() << std::endl;

                    // Probably a better way to do this comparison...
                    if (smallest.X() > cur_min.X())
                    {
                        smallest.X() = cur_min.X();
                    }
                    if (smallest.Y() > cur_min.Y())
                    {
                        smallest.Y() = cur_min.Y();
                    }
                    if (smallest.Z() > cur_min.Z())
                    {
                        smallest.Z() = cur_min.Z();
                    }

                    if (biggest.X() < cur_max.X())
                    {
                        biggest.X() = cur_max.X();
                    }
                    if (biggest.Y() < cur_max.Y())
                    {
                        biggest.Y() = cur_max.Y();
                    }
                    if (biggest.Z() < cur_max.Z())
                    {
                        biggest.Z() = cur_max.Z();
                    }
                }

                return true;
            }

            // Create AABB box since they are not created by default.
            else
            {
                std::cout << "No Bounding box found for model... Creating one" << std::endl;
                _ecem.CreateComponent(_entity, gz::sim::components::AxisAlignedBox());
                boxes_exist = false;
                return true;
            }
        });

    if (boxes_exist)
    {
        double x_size = ceil(biggest.X() - smallest.X());
        double y_size = ceil(biggest.Y() - smallest.Y());
        std::cout << "x_size " << x_size << " y_size " << y_size << std::endl;

        // Initialize occupancy grid based on the size of the environment. Going to make the size 5cm resolution fow now. Will likely be able to parameterize this later.
        double x_cells = ceil(x_size / this->GRID_SIZE) + this->PADDING;
        double y_cells = ceil(y_size / this->GRID_SIZE) + this->PADDING;
        this->occupancy_grid.resize(y_cells * x_cells, 0);
        std::cout << "x_grid_size " << x_cells << " y_grid_size " << y_cells << std::endl;
        this->grid_width = int(x_cells);
        this->grid_height = int(y_cells);
        this->size_init = true;
    }
}