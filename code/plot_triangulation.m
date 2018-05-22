function [  ] = plot_triangulation( triangulate_points, cameracenter1, cameracenter2 )

    figure; 
    axis equal;  
    hold on; 
    plot3(-triangulate_points(:,1), triangulate_points(:,2), triangulate_points(:,3), '.r');
    plot3(-cameracenter1(1), cameracenter1(2), cameracenter1(3),'*g');
    plot3(-cameracenter2(1), cameracenter2(2), cameracenter2(3),'*b');
    rotate3d on;
    grid on;
    xlabel('x');
    ylabel('y');
    zlabel('z');
    
end