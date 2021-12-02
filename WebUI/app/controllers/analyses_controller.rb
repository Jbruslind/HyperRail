require 'csv'

class AnalysesController < ApplicationController
  before_action :set_analysis, only: %i[ show aggregate individual edit update destroy ]

  # GET /analyses or /analyses.json
  def index
    @analyses = Analysis.all
  end

  # GET /analyses/1 or /analyses/1.json
  def show
  end

  def aggregate
    # Generate a CSV string with headers
    csv_string = CSV.generate(headers: true) do |csv|
      csv << ['Timestamp', 'Temperature', 'Humidity', 'Luminosity']

      # Find the first program (should only be one) and return all averaged data
      # TODO: Do this in one query instead of many
      # TODO: Exclude pieces that are not in our area of interest from the aggregates
      Program.first.runs.where('finished_at IS NOT NULL').each do |run|
        csv << [
          run.finished_at,
          run.aggregate_for('temperature'),
          run.aggregate_for('humidity'),
          run.aggregate_for('lux')
        ]
      end
    end

    send_data csv_string, type: Mime[:csv], disposition: "attachment; filename=aggregate-#{@analysis.id}.csv"
  end

  def individual
    # Generate a CSV string with headers
    csv_string = CSV.generate(headers: true) do |csv|
      csv << ['Timestamp', 'X', 'Y', 'Temperature', 'Humidity', 'Luminosity']

      # Find the first program (should only be one) and return all averaged data
      # TODO: Do this in one query instead of many
      # TODO: Exclude pieces that are not in our area of interest from the aggregates
      Program.first.runs.where('finished_at IS NOT NULL').each do |run|
        run.run_waypoints.includes(:sensor_readings).each do |run_waypoint|
          csv << [
            run_waypoint.finished_at,
            run_waypoint.x,
            run_waypoint.y,
            run_waypoint.sensor_readings.detect { |r| r.sensor_name == 'temperature' }.value,
            run_waypoint.sensor_readings.detect { |r| r.sensor_name == 'humidity' }.value,
            run_waypoint.sensor_readings.detect { |r| r.sensor_name == 'lux' }.value,
          ]
        end
      end
    end

    send_data csv_string, type: Mime[:csv], disposition: "attachment; filename=individual-#{@analysis.id}.csv"
  end

  # GET /analyses/new
  def new
    @analysis = Analysis.new
  end

  # GET /analyses/1/edit
  def edit
  end

  # POST /analyses or /analyses.json
  def create
    respond_to do |format|
      begin
        @analysis = Analysis.new(analysis_params)

        Analysis.transaction do
          # Save our analysis, raising an exception if it can't be saved
          @analysis.save!

          # Turn out serialized areas of interest into a ruby hash
          areas_of_interest = JSON.parse(params['analysis']['areas_of_interest'])

          # $stderr.puts areas_of_interest.inspect
          # Go through each area of interest and save it
          areas_of_interest.each do |rect_id, dimensions|
            @analysis.areas_of_interest.create!(
              rectangle_id: rect_id,
              x: dimensions['x'].to_f,
              y: dimensions['y'].to_f,
              width: dimensions['width'].to_f,
              height: dimensions['height'].to_f
            )
          end
  
          format.html { redirect_to @analysis, notice: "Analysis was successfully created." }
          format.json { render :show, status: :created, location: @analysis }
        rescue
          format.html { render :new, status: :unprocessable_entity }
          format.json { render json: @analysis.errors, status: :unprocessable_entity }
        end
      end
    end
  end

  # PATCH/PUT /analyses/1 or /analyses/1.json
  def update
    respond_to do |format|
      begin
        Analysis.transaction do
          # Delete all of the old areas of interest
          @analysis.areas_of_interest.destroy_all

          # Update our analysis, raising an exception if it can't be saved
          @analysis.update!(analysis_params)

          # Turn out serialized areas of interest into a ruby hash
          areas_of_interest = JSON.parse(params['analysis']['areas_of_interest'])

          # Go through each area of interest and save it
          areas_of_interest.each do |rect_id, dimensions|
            @analysis.areas_of_interest.create!(
              rectangle_id: rect_id,
              x: dimensions['x'].to_f,
              y: dimensions['y'].to_f,
              width: dimensions['width'].to_f,
              height: dimensions['height'].to_f
            )
          end
        end

        format.html { redirect_to @analysis, notice: "Analysis was successfully updated." }
        format.json { render :show, status: :ok, location: @analysis }
      rescue
        format.html { render :edit, status: :unprocessable_entity }
        format.json { render json: @analysis.errors, status: :unprocessable_entity }
      end
    end
  
    # DELETE /analyses/1 or /analyses/1.json
    def destroy
      @analysis.destroy
      respond_to do |format|
        format.html { redirect_to analyses_url, notice: "Analysis was successfully destroyed." }
        format.json { head :no_content }
      end
    end
  
    private
      # Use callbacks to share common setup or constraints between actions.
      def set_analysis
        @analysis = Analysis.find(params[:id])
      end
  
      # Only allow a list of trusted parameters through.
      def analysis_params
        params.fetch(:analysis).permit(:name)
      end
  end