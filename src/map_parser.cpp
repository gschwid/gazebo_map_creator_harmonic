#include "map_parser.hpp"

GZ_ADD_PLUGIN(
    map_parser::MapParser,
    gz::sim::System,
    map_parser::MapParser::ISystemPostUpdate,
    map_parser::MapParser::ISystemPreUpdate,
    map_parser::MapParser::ISystemConfigure);

using namespace map_parser;

void MapParser::threadedRosCallback()
{
    rclcpp::spin(this->ros_node);
}

MapParser::~MapParser()
{

    rclcpp::shutdown();
    if (this->ros_thread.joinable())
    {
        this->ros_thread.join();
        std::cout << "We cleaned up ROS like a boss" << std::endl;
    }
}

void MapParser::getOccupancyGridService(const std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
    this->generate_grid = true;
    for (int8_t& num: this->occupancy_grid) num = 0;
    this->grid_generated = false;
    RCLCPP_INFO(this->ros_node->get_logger(), "Service call successful.. Generating Occupancy grid");
}

void MapParser::Configure(const gz::sim::Entity &_entity,
                          const std::shared_ptr<const sdf::Element> &_sdf,
                          gz::sim::EntityComponentManager &_ecm,
                          gz::sim::EventManager &_eventMgr)
{
    if (!rclcpp::ok())
    {
        rclcpp::init(0, nullptr);
    }
    this->ros_node = std::make_shared<rclcpp::Node>("map_parser");
    this->ros_thread = std::thread(&MapParser::threadedRosCallback, this);
    this->occupancy_pub = this->ros_node->create_publisher<nav_msgs::msg::OccupancyGrid>("/map_parser/occupancy_grid", 10);
    this->occupancy_service = this->ros_node->create_service<std_srvs::srv::Empty>("/occupancy_service", std::bind(&MapParser::getOccupancyGridService, this, _1, _2));
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
    // Service got called to generate grid
    if (this->generate_grid)
    {
        if (!this->size_init)
        {
            gzmsg << "Need map size to be initialized" << std::endl;
            return;
        }

        this->getObstacles(_ecem);
        this->grid_generated = true;
        this->generate_grid = false;
    }

    if (this->grid_generated)
    {
        nav_msgs::msg::OccupancyGrid grid;
        grid.data = this->occupancy_grid;
        grid.info.height = this->grid_height;
        grid.info.width = this->grid_width;
        grid.info.resolution = this->GRID_SIZE;
        grid.header.frame_id = "map";
        grid.info.origin = this->origin;
        this->occupancy_pub->publish(grid);
    }
}

void MapParser::getObstacles(const gz::sim::EntityComponentManager &_ecem)
{

    _ecem.Each<gz::sim::components::Collision, gz::sim::components::Geometry, gz::sim::components::Name>(
        [this, &_ecem](const gz::sim::Entity &_entity, const gz::sim::components::Collision *_model, const gz::sim::components::Geometry *_geom, const gz::sim::components::Name *name) -> bool
        {
            // Dont want to include ground plane
            if (name->Data() != "ground_plane")
            {

                // Get world pose of each collision object
                gz::math::Pose3d worldPose = gz::sim::worldPose(_entity, _ecem);
                gzmsg << "X: " << worldPose.Pos().X() << " Y: " << worldPose.Pos().Y() << " Z: " << worldPose.Pos().Z() << std::endl;

                // Get the size depending on what type of object we are looking at
                gz::math::Vector3d scale;

                if (_geom->Data().Type() == sdf::GeometryType::BOX)
                {

                    // Making a draft of the occupancy grid filling feature.
                    scale = _geom->Data().BoxShape()->Size();
                    gzmsg << "This is a box" << std::endl;

                    int x_inarray = int((worldPose.X() - (this->origin.position.x)) / GRID_SIZE);
                    int y_inarray = int((worldPose.Y() - (this->origin.position.y)) / GRID_SIZE);
                    double half_x = (scale.X() / 2.0) / GRID_SIZE;
                    double half_y = (scale.Y() / 2.0) / GRID_SIZE;

                    // Get local corners of box
                    gz::math::v7::Vector3d test{1,1,1};
                    gz::math::v7::Quaternion<double> rotation = worldPose.Rot();
                    gz::math::v7::Vector3d tl{-(scale.X() / 2.0), scale.Y() / 2.0, 0};
                    gz::math::v7::Vector3d bl{-(scale.X() / 2.0), -(scale.Y() / 2.0), 0};
                    gz::math::v7::Vector3d tr{scale.X() / 2.0, scale.Y() / 2.0, 0};
                    gz::math::v7::Vector3d br{scale.X() / 2.0, -(scale.Y() / 2.0), 0};
                    
                    // Rotate local coordinates into world coordinates
                    gz::math::v7::Vector3d wtl = (rotation * tl) + worldPose.Pos();
                    gz::math::v7::Vector3d wbl = (rotation * bl) + worldPose.Pos();
                    gz::math::v7::Vector3d wtr = (rotation * tr) + worldPose.Pos();
                    gz::math::v7::Vector3d wbr = (rotation * br) + worldPose.Pos();

                    // Get min and max and translate that to the occupancy grid
                    double min_x = std::min({wtl.X(), wbl.X(), wtr.X(), wbr.X()});
                    double min_y = std::min({wtl.Y(), wbl.Y(), wtr.Y(), wbr.Y()});
                    double max_x = std::max({wtl.X(), wbl.X(), wtr.X(), wbr.X()});
                    double max_y = std::max({wtl.Y(), wbl.Y(), wtr.Y(), wbr.Y()});
                    int x_start = (min_x - origin.position.x) / GRID_SIZE;
                    int x_end = (max_x - origin.position.x) / GRID_SIZE;
                    int y_start = (min_y - origin.position.y) / GRID_SIZE;
                    int y_end = (max_y - origin.position.y) / GRID_SIZE;

                    for (int i = x_start; i < x_end; i++) {
                        for (int j = y_start; j < y_end; j++) {
                            occupancy_grid[i + grid_width * j] = OCCUPIED;
                        }
                    }
                    
                    std::cout << "----------------\n" << wtl << " " << wbl << " " << wtr << " " << wbr << std::endl;

                    // int x_start = std::floor(x_inarray - half_x);
                    // int x_end = std::ceil(x_inarray + half_x);
                    // int y_start = std::floor(y_inarray - half_y);
                    // int y_end = std::ceil(y_inarray + half_y);

                    std::cout << x_start << " " << x_end << " " << y_start << " " << y_end << std::endl;

                    // for (int i = x_start; i < x_end; i++)
                    // {
                    //     for (int j = y_start; j < y_end; j++)
                    //     {
                    //         occupancy_grid[i + grid_width * j] = OCCUPIED;
                    //     }
                    // }
                    gzmsg << origin.position.x << " " << origin.position.y << std::endl;
                    gzmsg << "Name " << name->Data() << " Scale X: " << scale.X() << " Scale Y: " << scale.Y() << " Scale Z: " << scale.Z() << std::endl;
                    gzmsg << "x_inarray " << x_inarray << " y_inarray " << y_inarray << std::endl;
                }

                else if (_geom->Data().Type() == sdf::GeometryType::CYLINDER)
                {
                    scale.X() = _geom->Data().CylinderShape()->Radius() * 2;
                    scale.Y() = scale.X();
                    scale.Z() = _geom->Data().CylinderShape()->Length();
                    gzmsg << "This is a cylinder" << std::endl;
                }

                else if (_geom->Data().Type() == sdf::GeometryType::SPHERE)
                {
                    scale.X() = _geom->Data().SphereShape()->Radius() * 2;
                    scale.Y() = scale.X();
                    scale.Z() = scale.X();
                    gzmsg << "This is a sphere" << std::endl;
                }

                else if (_geom->Data().Type() == sdf::GeometryType::PLANE)
                {
                    scale.X() = _geom->Data().PlaneShape()->Size().X();
                    scale.Y() = _geom->Data().PlaneShape()->Size().Y();
                    scale.Z() = 0;
                    gzmsg << "This is a plane" << std::endl;
                }

                else if (_geom->Data().Type() == sdf::GeometryType::MESH)
                {

                    auto path = gz::sim::asFullPath(_geom->Data().MeshShape()->Uri(), _geom->Data().MeshShape()->FilePath());
                    gzmsg << _geom->Data().MeshShape()->FilePath() << " " << _geom->Data().MeshShape()->Uri() << std::endl;
                }

                else
                {
                    gzmsg << "This is unrecognized BRU" << std::endl;
                    return true;
                }

                gzmsg << "Name " << name->Data() << " Scale X: " << scale.X() << " Scale Y: " << scale.Y() << " Scale Z: " << scale.Z() << std::endl;
            }
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
        double x_cells = ceil(x_size / this->GRID_SIZE);
        double y_cells = ceil(y_size / this->GRID_SIZE);
        this->occupancy_grid.resize(y_cells * x_cells, 0);
        std::cout << "x_grid_size " << x_cells << " y_grid_size " << y_cells << std::endl;
        this->grid_width = int(x_cells);
        this->grid_height = int(y_cells);
        this->size_init = true;
        this->origin.position.x = smallest.X();
        this->origin.position.y = smallest.Y();
        this->origin.position.z = 0.0;
    }
}