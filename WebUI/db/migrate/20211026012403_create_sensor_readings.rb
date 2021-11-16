class CreateSensorReadings < ActiveRecord::Migration[6.1]
  def change
    create_table :sensor_readings do |t|
      t.integer :run_waypoint_id, null: false
      t.string :sensor_name, null: false
      t.numeric :value
      t.string :metadata, null: false, default: '{}'
      t.timestamp :created_at
    end

    add_foreign_key :sensor_readings, :run_waypoints, on_delete: :cascade
    add_index :sensor_readings, :run_waypoint_id
  end
end
