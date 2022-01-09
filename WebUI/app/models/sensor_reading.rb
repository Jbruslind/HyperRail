class SensorReading < ApplicationRecord
  belongs_to :run_waypoint
  has_one :waypoint, through: :run_waypoint

  validates :sensor_name, :value, presence: true
  validates :value, numericality: true

  def self.latest_for(sensor_name)
    where(sensor_name: sensor_name).order(created_at: :desc).first
  end
end
