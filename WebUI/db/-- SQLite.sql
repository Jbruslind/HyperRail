-- SQLite
SELECT program_runs.id, name, camera_name
FROM program_runs 
JOIN programs 
    ON program_runs.program_id = programs.id 
JOIN run_waypoints 
    ON run_waypoints.program_run_id = program_runs.id
JOIN camera_images
    ON run_waypoints.id = camera_images.run_waypoint_id
GROUP BY programs.name;