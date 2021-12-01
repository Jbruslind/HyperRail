class Program < ApplicationRecord
  has_many :runs, class_name: 'ProgramRun'
  has_many :waypoints, -> { order(position: :asc) }

  validates :name, presence: true

  # Create a Program using the settings for the greenhouse and default camera settings
  def self.create_from_settings(name: 'Default Program')
    # Check to make sure the required settings are present
    ['camera_height', 'camera_fov', 'rail_x', 'rail_y', 'camera_crop'].each do |setting|
      raise RuntimeError.new("#{setting} is missing") if Setting.fetch(setting).nil?
    end

    # Perform all oof the program generation in a transaction, rollback upon failure
    transaction do
      # Create a program
      program = create(name: name)

      # Create a default camera from the settings
      camera = Camera.from_settings

      # Iterate through each camera centroid and create a waypoint
      waypoint_count = 0
      camera.to_centroids(
        rail_x: Setting.fetch('rail_x').to_f,
        rail_y: Setting.fetch('rail_y').to_f,
        crop: Setting.fetch('camera_crop').to_f
      ).each do |row|
        # For each row of centroids, go through each centroid
        row.each do |x, y|
          # Create a waypoint at the X/Y coordinate with the 'image' action
          program.waypoints.create!(x: x, y: y, z: camera.height_m, position: waypoint_count, actions: ['image', 'temperature', 'humidity', 'lux'])
          waypoint_count += 1
        end
      end

      program
    end
  end

  def archived?
    archived_at.present?
  end
end
