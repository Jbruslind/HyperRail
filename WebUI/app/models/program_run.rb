class ProgramRun < ApplicationRecord
  belongs_to :program

  has_many :run_waypoints
  has_many :waypoints, through: :run_waypoints
  has_many :sensor_readings, through: :run_waypoints
  has_many :camera_images, through: :run_waypoints
end
