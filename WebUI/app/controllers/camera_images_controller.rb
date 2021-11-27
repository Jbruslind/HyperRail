class CameraImagesController < ApplicationController
  before_action :image_path_required
  before_action :file_path_required

  def show
    send_file @file_path, disposition: :inline
  end

  private

  # Make sure that the image path as defined in our settings exists, or return 404
  def image_path_required
    @image_path = Pathname.new(Setting.fetch('image_path'))

    unless @image_path.directory?
      Rails.logger.warn "Uninitialized Image Path Directory"
      head :not_found
    end
  end

  def file_path_required
    # Expand the path to ensure we don't try to break out of our image root
    @file_path = Pathname.new(File.expand_path(@image_path.join(params[:path])))

    unless @file_path.to_s.starts_with?(@image_path.to_s)
      Rails.logger.warn "Cannot find image at #{@file_path.to_s}"
      head :not_found
    end
  end
end
