function plot_momentum_envelope(H_vertices, H_body_hist)

figure; hold on; grid on;
title('RW Momentum Envelope (Body Frame)');
xlabel('H_x'); ylabel('H_y'); zlabel('H_z');

% Plot envelope vertices
plot3(H_vertices(1,:), H_vertices(2,:), H_vertices(3,:), 'ro', 'MarkerSize', 8);

% Plot actual momentum trajectory
plot3(H_body_hist(1,:), H_body_hist(2,:), H_body_hist(3,:), 'b', 'LineWidth', 1.5);

legend('Envelope vertices','Momentum trajectory');
axis equal;
end
