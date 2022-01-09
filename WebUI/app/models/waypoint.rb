class Waypoint < ApplicationRecord
  serialize :actions, JSON

  belongs_to :program

  has_many :run_waypoints
  has_many :camera_images, through: :run_waypoints
  has_many :sensor_readings, through: :run_waypoints

  validates :position, :program_id, presence: true
  validates :position, uniqueness: { scope: :program_id }
  validates :x, :y, :z, numericality: {greater_than_or_equal_to: 0, allow_nil: true}
  validates_with CoordinateValidator
end
