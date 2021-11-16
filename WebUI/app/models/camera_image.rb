class CameraImage < ApplicationRecord
  serialize :metadata, JSON

  belongs_to :run_waypoint_id
  has_one :waypoint, through: :run_waypoint

  validates :camera_name, :image_type, :path, presence: true
end
