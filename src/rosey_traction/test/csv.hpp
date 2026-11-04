// Minimal CSV reader for the golden test files (header row, comma separated, no quoting).

#ifndef ROSEY_TRACTION__TEST__CSV_HPP_
#define ROSEY_TRACTION__TEST__CSV_HPP_

#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace test_csv
{

struct Row
{
  std::map<std::string, std::string> cells;
  std::string str(const std::string & key) const {return cells.at(key);}
  double num(const std::string & key) const {return std::stod(cells.at(key));}
};

inline std::vector<std::string> split(const std::string & line)
{
  std::vector<std::string> out;
  std::stringstream ss(line);
  std::string tok;
  while (std::getline(ss, tok, ',')) {
    while (!tok.empty() && (tok.back() == '\r' || tok.back() == ' ')) {tok.pop_back();}
    out.push_back(tok);
  }
  return out;
}

inline std::vector<Row> read(const std::string & path)
{
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("cannot open " + path);
  }
  std::string line;
  std::getline(in, line);
  const auto header = split(line);
  std::vector<Row> rows;
  while (std::getline(in, line)) {
    if (line.empty() || line == "\r") {continue;}
    const auto vals = split(line);
    Row r;
    for (size_t i = 0; i < header.size() && i < vals.size(); ++i) {
      r.cells[header[i]] = vals[i];
    }
    rows.push_back(r);
  }
  return rows;
}

}  // namespace test_csv

#endif  // ROSEY_TRACTION__TEST__CSV_HPP_
