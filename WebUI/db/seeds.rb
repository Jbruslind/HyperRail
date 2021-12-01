# Clear out all existing data
Program.destroy_all
Analysis.destroy_all
Setting.destroy_all

# Create default settings
{
  'password' => 'default-password',
  'rail_x' => '20',
  'rail_y' => '5',
  'camera_height' => '1.75',
  'camera_fov' => '47',
  'camera_crop' => '25',
  'image_path' => Rails.root.join('camera_images').to_s
}.each do |name, value|
  Setting.create name: name, value: value
end

# Make the default image directory
FileUtils.mkdir_p Setting.fetch('image_path')

# Create a default program
program = Program.create_from_settings
program.last_run_at = 1.day.ago
program.save!

# Now create a number of program runs for each program
runs = 5
runs.times do |i|
  Program.transaction do
    run = program.runs.create!(
      started_at: program.last_run_at - (runs-i).days,
      finished_at: program.last_run_at - (runs-i).days + 1.hours
    )

    # For each run, create a set of program_run_waypoints
    program.waypoints.each.with_index do |waypoint, idx|
      run_waypoint = run.run_waypoints.create!(
        waypoint: waypoint,
        x: waypoint.x,
        y: waypoint.y,
        z: waypoint.z,
        finished_at: run.started_at + idx.minutes + 2
      )

      # For each run_waypoint, add some sensor readings
      waypoint.actions.each do |action|
        case action
        when 'temperature'
          run_waypoint.sensor_readings.create!(sensor_name: action, value: rand(24.0..27.0).round(1), created_at: run_waypoint.finished_at)
        when 'lux'
          run_waypoint.sensor_readings.create!(sensor_name: action, value: rand(5..120000), created_at: run_waypoint.finished_at)
        when 'humidity'
          run_waypoint.sensor_readings.create!(sensor_name: action, value: rand(30.0..95.0).round(1), created_at: run_waypoint.finished_at)
        when 'image'
          # TODO: Seed images here once we figure out how
        end
      end
    end
  end
end

# Create 4 Analyses
camera = Camera.from_settings
centroids = []
camera.to_centroids.each do |row|
  centroids.concat(row)
end
sizes = camera.unit_sizes

4.times do |i|
  Analysis.transaction do
    analysis = Analysis.create(name: "Analysis #{i+1}")

    # Create a random number of areas of interest
    rand(5..15).times do |j|
      sample = centroids.sample
      analysis.areas_of_interest.create(rectangle_id: "rect#{centroids.index(sample)+1}", x: sample[0], y: sample[1], width: sizes[:x_unit_size], height: sizes[:y_unit_size])
    end
  end
end
