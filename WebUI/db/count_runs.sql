SELECT COUNT(DISTINCT program_runs.id) AS run_count, program_runs.id AS real_id, "programs"."name", camera_name, programs.*, "program_runs"."finished_at" FROM "programs" LEFT OUTER JOIN program_runs
          ON program_runs.program_id = programs.id
        LEFT OUTER JOIN run_waypoints
          ON run_waypoints.program_run_id = program_runs.id
        LEFT OUTER JOIN camera_images
          ON run_waypoints.id = camera_images.run_waypoint_id 
 