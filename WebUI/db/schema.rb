# This file is auto-generated from the current state of the database. Instead
# of editing this file, please use the migrations feature of Active Record to
# incrementally modify your database, and then regenerate this schema definition.
#
# This file is the source Rails uses to define your schema when running `bin/rails
# db:schema:load`. When creating a new database, `bin/rails db:schema:load` tends to
# be faster and is potentially less error prone than running all of your
# migrations from scratch. Old migrations may fail to apply correctly if those
# migrations use external dependencies or application code.
#
# It's strongly recommended that you check this file into your version control system.

ActiveRecord::Schema.define(version: 2021_10_31_143759) do

  create_table "camera_images", force: :cascade do |t|
    t.integer "run_waypoint_id", null: false
    t.string "camera_name", null: false
    t.string "image_type", null: false
    t.string "uri", null: false
    t.string "metadata", default: "{}", null: false
    t.datetime "created_at"
    t.index ["run_waypoint_id"], name: "index_camera_images_on_run_waypoint_id"
  end

  create_table "program_runs", force: :cascade do |t|
    t.integer "program_id", null: false
    t.datetime "started_at", null: false
    t.datetime "finished_at"
    t.index ["program_id"], name: "index_program_runs_on_program_id"
  end

  create_table "programs", force: :cascade do |t|
    t.string "name", null: false
    t.datetime "last_run_at"
    t.datetime "archived_at"
  end

  create_table "run_waypoints", force: :cascade do |t|
    t.integer "program_run_id", null: false
    t.integer "waypoint_id", null: false
    t.decimal "x"
    t.decimal "y"
    t.decimal "z"
    t.datetime "finished_at"
    t.index ["program_run_id", "waypoint_id"], name: "index_run_waypoints_on_program_run_id_and_waypoint_id", unique: true
    t.index ["waypoint_id"], name: "index_run_waypoints_on_waypoint_id"
  end

  create_table "sensor_readings", force: :cascade do |t|
    t.integer "run_waypoint_id", null: false
    t.string "sensor_name", null: false
    t.decimal "value"
    t.string "metadata", default: "{}", null: false
    t.datetime "created_at"
    t.index ["run_waypoint_id"], name: "index_sensor_readings_on_run_waypoint_id"
  end

  create_table "settings", force: :cascade do |t|
    t.string "name", null: false
    t.string "value"
    t.datetime "created_at", precision: 6, null: false
    t.datetime "updated_at", precision: 6, null: false
    t.index ["name"], name: "index_settings_on_name", unique: true
  end

  create_table "waypoints", force: :cascade do |t|
    t.integer "program_id", null: false
    t.integer "position", null: false
    t.decimal "x"
    t.decimal "y"
    t.decimal "z"
    t.string "actions"
    t.index ["program_id", "position"], name: "index_waypoints_on_program_id_and_position", unique: true
  end

  add_foreign_key "camera_images", "run_waypoints", on_delete: :cascade
  add_foreign_key "program_runs", "programs", on_delete: :cascade
  add_foreign_key "run_waypoints", "program_runs", on_delete: :cascade
  add_foreign_key "run_waypoints", "waypoints", on_delete: :cascade
  add_foreign_key "sensor_readings", "run_waypoints", on_delete: :cascade
  add_foreign_key "waypoints", "programs", on_delete: :cascade
end
