class Setting < ApplicationRecord
  validates :name, presence: true, uniqueness: true

  # Fetch a setting or return a default value.
  # Settings are cached for 10 seconds by default.
  def self.fetch(name, default=nil, expires_in: 10.seconds, force: false)
    Rails.cache.fetch("settings/#{name}", expires_in: 10.seconds, force: false) do
      if setting = select(:value).where(name: name).first
        setting.value
      else
        default
      end
    end
  end
end
