import type { Route } from "./+types/home";
import { Link } from "react-router";

export function meta({ }: Route.MetaArgs) {
  return [
    { title: "Home | Crawler" },
  ];
}

export default function Home() {
  return (
    <main className="flex flex-col items-center p-5 pt-20 gap-6">
      <h1 className="text-4xl font-bold">Crawler</h1>
      <div className="flex flex-col items-center gap-1 text-lg text-center">
        <p>Welcome to the Crawler controller!</p>
        <p>There are different controls and visualizations available:</p>
        <div className="h-1"></div>
        <Link to="/manual-control" className="text-blue-500 hover:text-blue-400 transition"> -&gt; Manual Control</Link>
        <Link to="/manual-control" className="text-blue-500 hover:text-blue-400 transition"> -&gt; Automatic Control</Link>
        <Link to="/manual-control" className="text-blue-500 hover:text-blue-400 transition"> -&gt; Visualization</Link>
        <div className="h-3"></div>
        <p>Check out the full source code on GitHub:</p>
        <div className="h-1"></div>
        <a href="https://github.com/hector-crawler/crawler-controller" target="_blank" className="text-blue-500 hover:text-blue-400 transition"> -&gt; github.com/hector-crawler/crawler-controller</a>
      </div>
    </main>
  );
}
