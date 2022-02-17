class Analysis < ApplicationRecord
  has_one_attached :image
  
  has_many :areas_of_interest, class_name: 'AreaOfInterest'

  def self.get_sizes
    camera = Camera.from_settings
    sizes = camera.unit_sizes(
      rail_x: Setting.fetch('rail_x').to_f,
      rail_y: Setting.fetch('rail_y').to_f,
      crop: Setting.fetch('camera_crop').to_f
    )
    sizes
  end

  def self.get_image
    # @files = Dir.glob("#{Rails.root}/app/assets/images/*.png").select { |f| File.file?(f) }
    @files = Dir.entries("#{Rails.root}/app/assets/images/").select { |f| f.include? ".png" }
    @files.sort_by{ |f| File.ctime("#{Rails.root}/app/assets/images/#{f}") }
    puts @files
    file = @files.first
  end

end
