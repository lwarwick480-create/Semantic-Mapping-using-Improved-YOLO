from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    min_points_arg = DeclareLaunchArgument(
        'min_points_threshold',
        default_value='5.0',
        description='Minimum number of points required to create a rectangle'
    )
    
    min_area_arg = DeclareLaunchArgument(
        'min_area_threshold', 
        default_value='0.05',
        description='Minimum area (m²) for valid rectangles'
    )
    
    max_area_arg = DeclareLaunchArgument(
        'max_area_threshold',
        default_value='15.0', 
        description='Maximum area (m²) for valid rectangles'
    )
    
    use_pca_arg = DeclareLaunchArgument(
        'use_pca_fitting',
        default_value='true',
        description='Use PCA-based fitting (true) or min-bounding box (false)'
    )
    
    publish_debug_arg = DeclareLaunchArgument(
        'publish_debug',
        default_value='true',
        description='Publish debug visualization markers'
    )
    
    rectangle_height_arg = DeclareLaunchArgument(
        'rectangle_height',
        default_value='0.8',
        description='Height of 3D rectangle visualization (m)'
    )

    # Semantic rectangle fitter node
    semantic_rect_fitter_node = Node(
        package='semantic_rect_fitting',
        executable='semantic_rect_fitter',
        name='semantic_rect_fitter',
        output='screen',
        parameters=[{
            'min_points_threshold': LaunchConfiguration('min_points_threshold'),
            'min_area_threshold': LaunchConfiguration('min_area_threshold'), 
            'max_area_threshold': LaunchConfiguration('max_area_threshold'),
            'use_pca_fitting': LaunchConfiguration('use_pca_fitting'),
            'publish_debug': LaunchConfiguration('publish_debug'),
            'rectangle_height': LaunchConfiguration('rectangle_height')
        }],
        remappings=[
            ('/semantic_markers', '/semantic_markers'),
            ('/semantic_rectangles', '/semantic_rectangles'),
            ('/semantic_rectangles_debug', '/semantic_rectangles_debug')
        ]
    )

    return LaunchDescription([
        min_points_arg,
        min_area_arg, 
        max_area_arg,
        use_pca_arg,
        publish_debug_arg,
        rectangle_height_arg,
        semantic_rect_fitter_node
    ])
