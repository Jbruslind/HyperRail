class CreateRunWaypoints < ActiveRecord::Migration[6.1]
  def change
    create_table :run_waypoints do |t|
      t.integer :program_run_id, null: false
      t.integer :waypoint_id, null: false
      t.numeric :x
      t.numeric :y
      t.numeric :z
      t.timestamp :finished_at
    end

    add_foreign_key :run_waypoints, :program_runs, on_delete: :cascade
    add_foreign_key :run_waypoints, :waypoints, on_delete: :cascade

    add_index :run_waypoints, [:program_run_id, :waypoint_id], unique: true
    add_index :run_waypoints, :waypoint_id
  end
end
