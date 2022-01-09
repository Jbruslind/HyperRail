module AnalysesHelper
  def areas_of_interest_for(analysis)
    hash = {}
    analysis.areas_of_interest.map do |area|
      hash[area.rectangle_id] = { x: area.x, y: area.y, width: area.width, height: area.width }
    end

    hash.to_json
  end
end
