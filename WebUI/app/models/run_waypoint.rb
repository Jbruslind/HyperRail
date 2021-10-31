class RunWaypoint < ApplicationRecord
  belongs_to :program_run
  belongs_to :waypoint

  has_many :camera_images
  has_many :sensor_readings
  validates :x, :y, :z, numericality: {greater_than_or_equal_to: 0, allow_nil: true}
  validates_with CoordinateValidator
end
