class SensorReading < ApplicationRecord
  belongs_to :run_waypoint
  has_one :waypoint, through: :run_waypoint

  validates :sensor_name, :value, presence: true
  validates :value, numericality: true
end
