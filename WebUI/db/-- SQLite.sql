-- SQLite
SELECT program_runs.id, name FROM program_runs INNER JOIN programs ON program_runs.program_id = programs.id GROUP BY programs.name;