class Program < ApplicationRecord
  has_many :runs, class_name: 'ProgramRun'
  has_many :waypoints, -> { order(position: :asc) }

  validates :name, presence: true

  def archived?
    archived_at.present?
  end
end
