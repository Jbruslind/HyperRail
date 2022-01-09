class CreateCameraImages < ActiveRecord::Migration[6.1]
  def change
    create_table :camera_images do |t|
      t.integer :run_waypoint_id, null: false
      t.string :camera_name, null: false
      t.string :image_type, null: false
      t.string :uri, null: false
      t.string :metadata, null: false, default: '{}'
      t.timestamp :created_at
    end

    add_foreign_key :camera_images, :run_waypoints, on_delete: :cascade
    add_index :camera_images, :run_waypoint_id
  end
end
