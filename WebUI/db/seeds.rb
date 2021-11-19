# Clear out all existing data
Program.destroy_all
Analysis.destroy_all

# Create some programs
program1 = Program.create!(name: 'Program 1')
program2 = Program.create!(name: 'Program 2', last_run_at: Time.now)
program3 = Program.create!(name: 'Program 3', last_run_at: 5.days.ago, archived_at: 1.days.ago)

# Create a number of waypoints for each program
10.times do |i|
  program1.waypoints.create!(position: i, x: (i.div(2)*100), y: (i % 2) * 100, actions: ['image', 'temperature'])
  program2.waypoints.create!(position: i, x: (i.div(2)*100), y: (i % 2) * 100, actions: ['humidity', 'lux'])
end
10.times do |i|
  program3.waypoints.create!(position: i, x: (i.div(2)*100), y: (i % 2) * 100 + 200, actions: ['image', 'temperature', 'humidity', 'lux'])
end

# Now create a number of program runs for each program
runs = 5
runs.times do |i|
  [program2, program3].each do |program|
    run = program.runs.create!(
      started_at: program.last_run_at - (runs-i).days,
      finished_at: program.last_run_at - (runs-i).days + 1.hours
    )

    # For each run, create a set of program_run_waypoints
    program1.waypoints.each.with_index do |waypoint, idx|
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
4.times do |i|
  analysis = Analysis.create(name: "Analysis #{i+1}")

  # Create a random number of areas of interest
  rand(5..15).times do |j|
    analysis.areas_of_interest.create(x: ((j % 2) + i) * 10, y: (j.div(2)*10), width: 10, height: 10)
  end
end
