class ProgramsController < ApplicationController
  before_action :set_program, only: %i[ show edit update destroy ]

  # GET /programs or /programs.json
  def index
    @programs = Program.select('programs.*', '(SELECT COUNT(*) FROM program_runs WHERE program_runs.program_id=programs.id) AS run_count').order(started_at: :desc).load
    #@program2 = ProgramRun.select('program_runs.*', '(SELECT COUNT(*) FROM program_runs WHERE program_runs.program_id=programs.id) AS run_count2').order(started_at: :desc)
    #@program2 = @program2.joins(:waypoints).distinct
    #program2 = ProgramRun.select('program_runs.*', '(SELECT COUNT(*) FROM program_runs WHERE program_runs.program_id=programs.id) AS run_count').order(started_at: :desc)
    if !@programs.blank? 
      @program2 = Program.select('COUNT(DISTINCT program_runs.id) AS run_count', 
      'program_runs.id AS real_id', 
      'camera_name', 
      'programs.*', 
      'program_runs.finished_at').order(started_at: :desc)
      @program2 = @program2.joins('
        LEFT OUTER JOIN program_runs 
          ON program_runs.program_id = programs.id
        LEFT OUTER JOIN run_waypoints 
          ON run_waypoints.program_run_id = program_runs.id
        LEFT OUTER JOIN camera_images
          ON run_waypoints.id = camera_images.run_waypoint_id').load
    else
      @programs2 = []
    end
    puts @programs.inspect 
    # puts !@programs.blank? 
    puts @program2.inspect

    @files = Dir.glob("#{Rails.root}/camera_images/**/*.tif").sort_by{ |f| File.ctime(f) }
    @file_handler = File
  end

  # POST /progams/default
  def default
    program = Program.create_from_settings
    flash[:success] = "Successfully created a new default program"
    redirect_to programs_path
  end

  # GET /programs/1 or /programs/1.json
  def show
  end

  # GET /programs/new
  def new
    @program = Program.new
  end

  # GET /programs/1/edit
  def edit
  end

  # POST /programs or /programs.json
  def create
    @program = Program.new(program_params)

    respond_to do |format|
      if @program.save
        format.html { redirect_to @program, notice: "Program was successfully created." }
        format.json { render :show, status: :created, location: @program }
      else
        format.html { render :new, status: :unprocessable_entity }
        format.json { render json: @program.errors, status: :unprocessable_entity }
      end
    end
  end

  # PATCH/PUT /programs/1 or /programs/1.json
  def update
    respond_to do |format|
      if @program.update(program_params)
        format.html { redirect_to programs_path, notice: "Program was successfully updated." }
        format.json { render :show, status: :ok, location: @program }
      else
        format.html { render :edit, status: :unprocessable_entity }
        format.json { render json: @program.errors, status: :unprocessable_entity }
      end
    end
  end

  # DELETE /programs/1 or /programs/1.json
  def destroy
    @program.destroy
    respond_to do |format|
      format.html { redirect_to programs_url, notice: "Program was successfully destroyed." }
      format.json { head :no_content }
    end
  end

  def download_zip
    if !params[:camera_name].blank? && !params[:run].blank? 
      file_str = "#{Rails.root}/camera_images/" + params[:run] + "_" + params[:camera_name] + ".zip"
      send_file file_str, type: "application/zip", x_sendfile: true
    else
      respond_to do |format|
        format.html { redirect_to programs_url, notice: "No recent images found" }
        format.json { head :no_content }
      end
    end
  end

  private

  # Use callbacks to share common setup or constraints between actions.
  def set_program
    @program = Program.find(params[:id])
  end

  # Only allow a list of trusted parameters through.
  def program_params
    params.fetch(:program, {}).permit!
  end
  
end
